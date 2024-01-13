/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file navigator_mission.cpp
 *
 * Helper class to access missions
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Simon Wilks <simon@uaventure.com>
 * @author Andreas Antener <andreas@uaventure.com>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "mission.h"
#include "navigator.h"

#include <string.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <lib/geo/geo.h>
#include <navigator/navigation.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>

using namespace time_literals;

static constexpr int32_t DEFAULT_MISSION_CACHE_SIZE = 10;

Mission::Mission(Navigator *navigator) :
	MissionBase(navigator, DEFAULT_MISSION_CACHE_SIZE)
{
}

void
Mission::on_inactive()
{
	_vehicle_status_sub.update();

	if (_need_mission_save && _vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		save_mission_state();
	}

	MissionBase::on_inactive();
}

void
Mission::on_activation()
{
	_need_mission_save = true;

	MissionBase::on_activation();
}

bool
Mission::isLanding()
{
	if (get_land_start_available()) {
		static constexpr size_t max_num_next_items{1u};
		int32_t next_mission_items_index[max_num_next_items];
		size_t num_found_items;

		getNextPositionItems(_mission.land_start_index + 1, next_mission_items_index, num_found_items, max_num_next_items);

		// vehicle is currently landing if
		//  mission valid, still flying, and in the landing portion of mission (past land start marker)
		bool on_landing_stage = (num_found_items > 0U) &&  _mission.current_seq > next_mission_items_index[0U];

		// special case: if the land start index is at a LOITER_TO_ALT WP, then we're in the landing sequence already when the
		// distance to the WP is below the loiter radius + acceptance.
		if ((num_found_items > 0U) && _mission.current_seq == next_mission_items_index[0U]
		    && _mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT) {
			const float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
						_navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

			// consider mission_item.loiter_radius invalid if NAN or 0, use default value in this case.
			const float mission_item_loiter_radius_abs = (PX4_ISFINITE(_mission_item.loiter_radius)
					&& fabsf(_mission_item.loiter_radius) > FLT_EPSILON) ? fabsf(_mission_item.loiter_radius) :
					_navigator->get_loiter_radius();

			on_landing_stage = d_current <= (_navigator->get_acceptance_radius() + mission_item_loiter_radius_abs);
		}

		return _navigator->get_mission_result()->valid && on_landing_stage;

	} else {
		return false;
	}
}

bool
Mission::set_current_mission_index(uint16_t index)
{
	if (index == _mission.current_seq) {
		return true;
	}

	if (_navigator->get_mission_result()->valid && (index < _mission.count)) {
		if (goToItem(index, true) != PX4_OK) {
			// Keep the old mission index (it was not updated by the interface) and report back.
			return false;
		}

		_is_current_planned_mission_item_valid = true;

		// we start from the first item so can reset the cache
		if (_mission.current_seq == 0) {
			resetItemCache();
		}

		// update mission items if already in active mission
		if (isActive()) {
			// prevent following "previous - current" line
			_navigator->reset_triplets();
			update_mission();
			set_mission_items();
		}

		// User has actively set new index, reset.
		_inactivation_index = -1;

		return true;
	}

	return false;
}

bool Mission::setNextMissionItem()
{
	return (goToNextItem(true) == PX4_OK);
}

bool
Mission::do_need_move_to_takeoff()
{
	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _global_pos_sub.get().lat, _global_pos_sub.get().lon);

		return d_current > _navigator->get_acceptance_radius();
	}

	return false;
}

void Mission::setActiveMissionItems()
{
	/* Get mission item that comes after current if available */
	static constexpr size_t max_num_next_items{2u};
	int32_t next_mission_items_index[max_num_next_items];
	size_t num_found_items;

	getNextPositionItems(_mission.current_seq + 1, next_mission_items_index, num_found_items, max_num_next_items);

	mission_item_s next_mission_items[max_num_next_items];
	const dm_item_t dataman_id = static_cast<dm_item_t>(_mission.dataman_id);

	for (size_t i = 0U; i < num_found_items; i++) {
		mission_item_s next_mission_item;
		bool success = _dataman_cache.loadWait(dataman_id, next_mission_items_index[i],
						       reinterpret_cast<uint8_t *>(&next_mission_item), sizeof(next_mission_item), MAX_DATAMAN_LOAD_WAIT);

		if (success) {
			next_mission_items[i] = next_mission_item;

		} else {
			num_found_items = i;
			break;
		}
	}

	/*********************************** handle mission item *********************************************/
	WorkItemType new_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	const position_setpoint_s current_setpoint_copy = pos_sp_triplet->current;

	if (item_contains_position(_mission_item)) {

		handleTakeoff(new_work_item_type, next_mission_items, num_found_items);

		handleLanding(new_work_item_type, next_mission_items, num_found_items);

		// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
		if (new_work_item_type != WorkItemType::WORK_ITEM_TYPE_PRECISION_LAND) {
			mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		}

		// Allow a rotary wing vehicle to decelerate before reaching a wp with a hold time or a timeout
		// This is done by setting the position triplet's next position's valid flag to false,
		// which makes the FlightTask disregard the next position
		// TODO: Setting the next waypoint's validity flag to handle braking / correct waypoint behavior
		// seems hacky, handle this more properly.
		const bool brake_for_hold = _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					    && (get_time_inside(_mission_item) > FLT_EPSILON || item_has_timeout(_mission_item));

		if (_mission_item.autocontinue && !brake_for_hold) {
			/* try to process next mission item */
			if (num_found_items >= 1u) {
				/* got next mission item, update setpoint triplet */
				mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->next);

			} else {
				/* next mission item is not available */
				pos_sp_triplet->next.valid = false;
			}

		} else {
			/* vehicle will be paused on current waypoint, don't set next item */
			pos_sp_triplet->next.valid = false;
		}

	} else if (item_contains_gate(_mission_item)) {
		// The mission item is a gate, let's check if the next item in the list provides
		// a position to go towards.

		if (num_found_items > 0u) {
			// We have a position, convert it to the setpoint and update setpoint triplet
			mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->current);
		}

		if (num_found_items >= 2u) {
			/* got next mission item, update setpoint triplet */
			mission_item_to_position_setpoint(next_mission_items[1u], &pos_sp_triplet->next);

		} else {
			pos_sp_triplet->next.valid = false;
		}

	} else {
		handleVtolTransition(new_work_item_type, next_mission_items, num_found_items);
	}

	// Only set the previous position item if the current one really changed
	if ((_work_item_type != WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND) &&
	    !position_setpoint_equal(&pos_sp_triplet->current, &current_setpoint_copy)) {
		pos_sp_triplet->previous = current_setpoint_copy;
	}

	issue_command(_mission_item);

	/* set current work item type */
	_work_item_type = new_work_item_type;

	reset_mission_item_reached();

	if (_mission_type == MissionType::MISSION_TYPE_MISSION) {
		set_mission_result();
	}

	publish_navigator_mission_item(); // for logging
	_navigator->set_position_setpoint_triplet_updated();
}

void Mission::handleTakeoff(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
			    size_t &num_found_items)
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* do climb before going to setpoint if needed and not already executing climb */
	/* in fixed-wing this whole block will be ignored and a takeoff item is always propagated */
	if (PX4_ISFINITE(_mission_init_climb_altitude_amsl) &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_CLIMB;

		/* use current mission item as next position item */
		num_found_items = 1u;
		next_mission_items[0u] = _mission_item;
		next_mission_items[0u].nav_cmd = NAV_CMD_WAYPOINT;

		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Climb to %.1f meters above home\t",
				 (double)(_mission_init_climb_altitude_amsl - _navigator->get_home_position()->alt));
		events::send<float>(events::ID("mission_climb_before_start"), events::Log::Info,
				    "Climb to {1:.1m_v} above home", _mission_init_climb_altitude_amsl - _navigator->get_home_position()->alt);

		if (_land_detected_sub.get().landed) {
			_mission_item.nav_cmd = NAV_CMD_TAKEOFF;

		} else {
			_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
		}

		_mission_item.lat = _global_pos_sub.get().lat;
		_mission_item.lon = _global_pos_sub.get().lon;
		_mission_item.yaw = NAN; // FlightTaskAuto handles yaw directly
		_mission_item.altitude = _mission_init_climb_altitude_amsl;
		_mission_item.altitude_is_relative = false;
		_mission_item.autocontinue = true;
		_mission_item.time_inside = 0.0f;

		_mission_init_climb_altitude_amsl = NAN;

	} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
		   && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT
		   && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

		/* if there is no need to do a takeoff but we have a takeoff item, treat is as waypoint */
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
		_mission_item.yaw = NAN;

	} else if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
		   && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {
		// if the vehicle is already in fixed wing mode then the current mission item
		// will be accepted immediately and the work items will be skipped
		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_CLIMB;


		/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
		_mission_item.yaw = NAN;
	}

	/* if we just did a normal takeoff navigate to the actual waypoint now */
	if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_CLIMB) {

		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
		_mission_item.yaw = NAN;
	}

	/* if we just did a VTOL takeoff, prepare transition */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_CLIMB &&
	    _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
	    !_land_detected_sub.get().landed) {

		/* set yaw setpoint to heading of VTOL_TAKEOFF wp against current position */
		_mission_item.yaw = get_bearing_to_next_waypoint(
					    _global_pos_sub.get().lat, _global_pos_sub.get().lon,
					    _mission_item.lat, _mission_item.lon);

		_mission_item.force_heading = true;

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING;

		/* set position setpoint to current while aligning */
		_mission_item.lat = _global_pos_sub.get().lat;
		_mission_item.lon = _global_pos_sub.get().lon;
	}

	/* heading is aligned now, prepare transition */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING &&
	    _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
	    !_land_detected_sub.get().landed) {

		/* check if the vtol_takeoff waypoint is on top of us */
		if (do_need_move_to_takeoff()) {
			new_work_item_type = WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF;
		}

		set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
		_mission_item.yaw = NAN;

		// keep current setpoints (FW position controller generates wp to track during transition)
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	}

	/* takeoff completed and transitioned, move to takeoff wp as fixed wing */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
	    && _work_item_type == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission_item.autocontinue = true;
		_mission_item.time_inside = 0.0f;
	}
}

void Mission::handleVtolTransition(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
				   size_t &num_found_items)
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* turn towards next waypoint before MC to FW transition */
	if (_mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
	    && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT
	    && new_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT
	    && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && !_land_detected_sub.get().landed
	    && (num_found_items > 0u)) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING;

		set_align_mission_item(&_mission_item, &next_mission_items[0u]);

		/* set position setpoint to target during the transition */
		mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->current);
	}

	/* yaw is aligned now */
	if (_work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING &&
	    new_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;

		pos_sp_triplet->previous = pos_sp_triplet->current;
		// keep current setpoints (FW position controller generates wp to track during transition)
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	}
}

void
Mission::save_mission_state()
{
	if (_vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		// Save only while disarmed, as this is a blocking operation
		_need_mission_save = true;
		return;
	}

	_need_mission_save = false;
	mission_s mission_state = {};

	/* read current state */
	bool success = _dataman_client.readSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission_state),
						sizeof(mission_s));

	if (success) {
		/* data read successfully, check dataman ID and items count */
		if (mission_state.dataman_id == _mission.dataman_id && mission_state.count == _mission.count
		    && mission_state.mission_id == _mission.mission_id) {
			/* navigator may modify only sequence, write modified state only if it changed */
			if (mission_state.current_seq != _mission.current_seq) {
				mission_state = _mission;

				success = _dataman_client.writeSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission_state),
								    sizeof(mission_s));

				if (!success) {

					PX4_ERR("Can't save mission state");
				}
			}
		}

	} else {
		/* invalid data, this must not happen and indicates error in mission publisher */
		mission_state = _mission;

		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Invalid mission state.\t");
		/* EVENT
		 * @description No mission or storage failure
		 */
		events::send(events::ID("mission_invalid_mission_state"), events::Log::Error, "Invalid mission state");

		/* write modified state only if changed */
		success = _dataman_client.writeSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission_state),
						    sizeof(mission_s));

		if (!success) {

			PX4_ERR("Can't save mission state");
		}
	}
<<<<<<< HEAD

	/*********************************** handle mission item *********************************************/

	/* handle mission items depending on the mode */

	const position_setpoint_s current_setpoint_copy = _navigator->get_position_setpoint_triplet()->current;

	if (item_contains_position(_mission_item)) {
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD: {
				/* force vtol land */
				if (_navigator->force_vtol() && _mission_item.nav_cmd == NAV_CMD_LAND) {
					_mission_item.nav_cmd = NAV_CMD_VTOL_LAND;
				}

				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

				/* do takeoff before going to setpoint if needed and not already in takeoff */
				/* in fixed-wing this whole block will be ignored and a takeoff item is always propagated */
				if (do_need_vertical_takeoff() &&
				    _work_item_type == WORK_ITEM_TYPE_DEFAULT &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_TAKEOFF;

					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					mission_item_next_position.nav_cmd = NAV_CMD_WAYPOINT;
					has_next_position_item = true;

					float takeoff_alt = calculate_takeoff_altitude(&_mission_item);

					mavlink_log_info(_navigator->get_mavlink_log_pub(), "Takeoff to %.1f meters above home\t",
							 (double)(takeoff_alt - _navigator->get_home_position()->alt));
					events::send<float>(events::ID("mission_takeoff_to"), events::Log::Info,
							    "Takeoff to {1:.1m_v} above home", takeoff_alt - _navigator->get_home_position()->alt);

					_mission_item.nav_cmd = NAV_CMD_TAKEOFF;
					_mission_item.lat = _navigator->get_global_position()->lat;
					_mission_item.lon = _navigator->get_global_position()->lon;
					/* hold heading for takeoff items */
					_mission_item.yaw = _navigator->get_local_position()->heading;
					_mission_item.altitude = takeoff_alt;
					_mission_item.altitude_is_relative = false;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;

				} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
					   && _work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

					/* if there is no need to do a takeoff but we have a takeoff item, treat is as waypoint */
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;

				} else if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
					   && _work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {
					// if the vehicle is already in fixed wing mode then the current mission item
					// will be accepted immediately and the work items will be skipped
					_work_item_type = WORK_ITEM_TYPE_TAKEOFF;


					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;
				}

				/* if we just did a normal takeoff navigate to the actual waypoint now */
				if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF &&
				    _work_item_type == WORK_ITEM_TYPE_TAKEOFF &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;
				}

				/* if we just did a VTOL takeoff, prepare transition */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
				    _work_item_type == WORK_ITEM_TYPE_TAKEOFF &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT &&
				    _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
				    !_navigator->get_land_detected()->landed) {

					/* disable weathervane before front transition for allowing yaw to align */
					pos_sp_triplet->current.disable_weather_vane = true;

					/* set yaw setpoint to heading of VTOL_TAKEOFF wp against current position */
					_mission_item.yaw = get_bearing_to_next_waypoint(
								    _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
								    _mission_item.lat, _mission_item.lon);

					_mission_item.force_heading = true;

					new_work_item_type = WORK_ITEM_TYPE_ALIGN;

					/* set position setpoint to current while aligning */
					_mission_item.lat = _navigator->get_global_position()->lat;
					_mission_item.lon = _navigator->get_global_position()->lon;
				}

				/* heading is aligned now, prepare transition */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
				    _work_item_type == WORK_ITEM_TYPE_ALIGN &&
				    _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
				    !_navigator->get_land_detected()->landed) {

					/* re-enable weather vane again after alignment */
					pos_sp_triplet->current.disable_weather_vane = false;

					/* check if the vtol_takeoff waypoint is on top of us */
					if (do_need_move_to_takeoff()) {
						new_work_item_type = WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF;
					}

					set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
					_mission_item.yaw = _navigator->get_local_position()->heading;

					// keep current setpoints (FW position controller generates wp to track during transition)
					pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
				}

				/* takeoff completed and transitioned, move to takeoff wp as fixed wing */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
				    && _work_item_type == WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_DEFAULT;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				/* move to land wp as fixed wing */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_LAND
				    && (_work_item_type == WORK_ITEM_TYPE_DEFAULT || _work_item_type == WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF)
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && !_navigator->get_land_detected()->landed) {

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND;

					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					has_next_position_item = true;

					float altitude = _navigator->get_global_position()->alt;

					if (pos_sp_triplet->current.valid && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
						altitude = pos_sp_triplet->current.alt;
					}

					_mission_item.altitude = altitude;
					_mission_item.altitude_is_relative = false;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
					_mission_item.vtol_back_transition = true;
				}

				/* transition to MC */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_LAND
				    && _work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				    && !_navigator->get_land_detected()->landed) {

					set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
					_mission_item.altitude = _navigator->get_global_position()->alt;
					_mission_item.altitude_is_relative = false;

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION;

					// make previous setpoint invalid, such that there will be no prev-current line following
					// if the vehicle drifted off the path during back-transition it should just go straight to the landing point
					pos_sp_triplet->previous.valid = false;
				}

				/* move to landing waypoint before descent if necessary */
				if (do_need_move_to_land() &&
				    (_work_item_type == WORK_ITEM_TYPE_DEFAULT ||
				     _work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION) &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND;

					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					has_next_position_item = true;

					/*
					 * Ignoring waypoint altitude:
					 * Set altitude to the same as we have now to prevent descending too fast into
					 * the ground. Actual landing will descend anyway until it touches down.
					 * XXX: We might want to change that at some point if it is clear to the user
					 * what the altitude means on this waypoint type.
					 */
					float altitude = _navigator->get_global_position()->alt;

					if (pos_sp_triplet->current.valid
					    && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
						altitude = pos_sp_triplet->current.alt;
					}

					_mission_item.altitude = altitude;
					_mission_item.altitude_is_relative = false;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;

					// have to reset here because these field were used in set_vtol_transition_item
					_mission_item.time_inside = 0.f;
					_mission_item.acceptance_radius = _navigator->get_acceptance_radius();

					// make previous setpoint invalid, such that there will be no prev-current line following.
					// if the vehicle drifted off the path during back-transition it should just go straight to the landing point
					pos_sp_triplet->previous.valid = false;

				} else if (_mission_item.nav_cmd == NAV_CMD_LAND && _work_item_type == WORK_ITEM_TYPE_DEFAULT) {
					if (_mission_item.land_precision > 0 && _mission_item.land_precision < 3) {
						new_work_item_type = WORK_ITEM_TYPE_PRECISION_LAND;

						if (_mission_item.land_precision == 1) {
							_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);

						} else { //_mission_item.land_precision == 2
							_navigator->get_precland()->set_mode(PrecLandMode::Required);
						}

						_navigator->get_precland()->on_activation();

					}
				}

				/* we just moved to the landing waypoint, now descend */
				if (_work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					if (_mission_item.land_precision > 0 && _mission_item.land_precision < 3) {
						new_work_item_type = WORK_ITEM_TYPE_PRECISION_LAND;

						if (_mission_item.land_precision == 1) {
							_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);

						} else { //_mission_item.land_precision == 2
							_navigator->get_precland()->set_mode(PrecLandMode::Required);
						}

						_navigator->get_precland()->on_activation();

					}

				}

				/* ignore yaw for landing items */
				/* XXX: if specified heading for landing is desired we could add another step before the descent
				 * that aligns the vehicle first */
				if (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND) {
					_mission_item.yaw = NAN;
				}


				// for fast forward convert certain types to simple waypoint
				// XXX: add other types which should be ignored in fast forward
				if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD &&
				    ((_mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED) ||
				     (_mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT))) {
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				break;
			}

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE: {
				if (item_contains_position(_mission_item)) {
					// convert mission item to a simple waypoint
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;

				} else {
					mavlink_log_critical(_navigator->get_mavlink_log_pub(),
							     "MissionReverse: Got a non-position mission item, ignoring it\t");
					events::send(events::ID("mission_ignore_non_position_item"), events::Log::Info,
						     "MissionReverse: Got a non-position mission item, ignoring it");
				}

				break;
			}
		}

	} else {
		/* handle non-position mission items such as commands */
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD: {
				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

				/* turn towards next waypoint before MC to FW transition */
				if (_mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
				    && _work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				    && !_navigator->get_land_detected()->landed
				    && has_next_position_item) {

					/* disable weathervane before front transition for allowing yaw to align */
					pos_sp_triplet->current.disable_weather_vane = true;

					new_work_item_type = WORK_ITEM_TYPE_ALIGN;

					set_align_mission_item(&_mission_item, &mission_item_next_position);

					/* set position setpoint to target during the transition */
					mission_apply_limitation(_mission_item);
					mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->current);
				}

				/* yaw is aligned now */
				if (_work_item_type == WORK_ITEM_TYPE_ALIGN &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

					/* re-enable weather vane again after alignment */
					pos_sp_triplet->current.disable_weather_vane = false;

					pos_sp_triplet->previous = pos_sp_triplet->current;
					// keep current setpoints (FW position controller generates wp to track during transition)
					pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
				}

				// ignore certain commands in mission fast forward
				if ((_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD) &&
				    (_mission_item.nav_cmd == NAV_CMD_DELAY)) {
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				break;
			}

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE: {
				// nothing to do, all commands are ignored
				break;
			}
		}

		if (_mission_item.nav_cmd == NAV_CMD_CONDITION_GATE) {
			_mission_item.autocontinue = true;
			_mission_item.time_inside = 0;
		}
	}

	/*********************************** set setpoints and check next *********************************************/
	// The logic in this section establishes the tracking between the current waypoint
	// which we are approaching and the next waypoint, which will tell us in which direction
	// we will change our trajectory right after reaching it.

	// Because actions, gates and jump labels can be interleaved with waypoints,
	// we are searching around the current mission item in the list to find the closest
	// gate and the closest waypoint. We then store them separately.

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Check if the mission item is a gate along the current trajectory
	if (item_contains_gate(_mission_item)) {

		// The mission item is a gate, let's check if the next item in the list provides
		// a position to go towards.

		// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
		if (has_next_position_item) {
			// We have a position, convert it to the setpoint and update setpoint triplet
			mission_apply_limitation(mission_item_next_position);
			mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->current);
		}

		// ELSE: The current position setpoint stays unchanged.

	} else {
		// The mission item is not a gate, set the current position setpoint from mission item (is protected against non-position items)
		// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
		if (new_work_item_type != WORK_ITEM_TYPE_PRECISION_LAND) {
			mission_apply_limitation(_mission_item);
			mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		}

		// ELSE: The current position setpoint stays unchanged.
	}

	// Only set the previous position item if the current one really changed
	// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
	if ((_work_item_type != WORK_ITEM_TYPE_MOVE_TO_LAND) &&
	    !position_setpoint_equal(&pos_sp_triplet->current, &current_setpoint_copy)) {
		pos_sp_triplet->previous = current_setpoint_copy;
	}

	/* issue command if ready (will do nothing for position mission items) */
	issue_command(_mission_item);

	/* set current work item type */
	_work_item_type = new_work_item_type;

	/* require takeoff after landing or idle */
	if (pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

		_need_takeoff = true;
	}

	_navigator->set_can_loiter_at_sp(false);
	reset_mission_item_reached();

	if (_mission_type == MISSION_TYPE_MISSION) {
		set_current_mission_item();
	}

	// If the mission item under evaluation contains a gate, we need to check if we have a next position item so
	// the controller can fly the correct line between the current and next setpoint
	if (item_contains_gate(_mission_item)) {
		if (has_after_next_position_item) {
			/* got next mission item, update setpoint triplet */
			mission_apply_limitation(mission_item_next_position);
			mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->next);

		} else {
			pos_sp_triplet->next.valid = false;
		}

	} else {
		// Allow a rotary wing vehicle to decelerate before reaching a wp with a hold time or a timeout
		// This is done by setting the position triplet's next position's valid flag to false,
		// which makes the FlightTask disregard the next position
		// TODO: Setting the next waypoint's validity flag to handle braking / correct waypoint behavior
		// seems hacky, handle this more properly.
		const bool brake_for_hold = _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					    && (get_time_inside(_mission_item) > FLT_EPSILON || item_has_timeout(_mission_item));

		if (_mission_item.autocontinue && !brake_for_hold) {
			/* try to process next mission item */
			if (has_next_position_item) {
				/* got next mission item, update setpoint triplet */
				mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->next);

			} else {
				/* next mission item is not available */
				pos_sp_triplet->next.valid = false;
			}

		} else {
			/* vehicle will be paused on current waypoint, don't set next item */
			pos_sp_triplet->next.valid = false;
		}
	}

	publish_navigator_mission_item(); // for logging
	_navigator->set_position_setpoint_triplet_updated();
}

bool
Mission::do_need_vertical_takeoff()
{
	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

		float takeoff_alt = calculate_takeoff_altitude(&_mission_item);

		if (_navigator->get_land_detected()->landed) {
			/* force takeoff if landed (additional protection) */
			_need_takeoff = true;

		} else if (_navigator->get_global_position()->alt > takeoff_alt - _navigator->get_altitude_acceptance_radius()) {
			/* if in-air and already above takeoff height, don't do takeoff */
			_need_takeoff = false;

		} else if (_navigator->get_global_position()->alt <= takeoff_alt - _navigator->get_altitude_acceptance_radius()
			   && (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
			       || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)) {
			/* if in-air but below takeoff height and we have a takeoff item */
			_need_takeoff = true;
		}

		/* check if current mission item is one that requires takeoff before */
		if (_need_takeoff && (
			    _mission_item.nav_cmd == NAV_CMD_TAKEOFF ||
			    _mission_item.nav_cmd == NAV_CMD_WAYPOINT ||
			    _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED)) {

			_need_takeoff = false;
			return true;
		}
	}

	return false;
}

bool
Mission::do_need_move_to_land()
{
	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND)) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

		return d_current > _navigator->get_acceptance_radius();
	}

	return false;
}

bool
Mission::do_need_move_to_takeoff()
{
	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

		return d_current > _navigator->get_acceptance_radius();
	}

	return false;
}

void
Mission::copy_position_if_valid(struct mission_item_s *mission_item, struct position_setpoint_s *setpoint)
{
	if (setpoint->valid && setpoint->type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
		mission_item->lat = setpoint->lat;
		mission_item->lon = setpoint->lon;
		mission_item->altitude = setpoint->alt;

	} else {
		mission_item->lat = _navigator->get_global_position()->lat;
		mission_item->lon = _navigator->get_global_position()->lon;
		mission_item->altitude = _navigator->get_global_position()->alt;
	}

	mission_item->altitude_is_relative = false;
}

void
Mission::set_align_mission_item(struct mission_item_s *mission_item, struct mission_item_s *mission_item_next)
{
	mission_item->nav_cmd = NAV_CMD_WAYPOINT;
	copy_position_if_valid(mission_item, &(_navigator->get_position_setpoint_triplet()->current));
	mission_item->altitude_is_relative = false;
	mission_item->autocontinue = true;
	mission_item->time_inside = 0.0f;
	mission_item->yaw = get_bearing_to_next_waypoint(
				    _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
				    mission_item_next->lat, mission_item_next->lon);
	mission_item->force_heading = true;
}

float
Mission::calculate_takeoff_altitude(struct mission_item_s *mission_item)
{
	/* calculate takeoff altitude */
	float takeoff_alt = get_absolute_altitude_for_item(*mission_item);

	/* takeoff to at least MIS_TAKEOFF_ALT above home/ground, even if first waypoint is lower */
	if (_navigator->get_land_detected()->landed) {
		takeoff_alt = fmaxf(takeoff_alt, _navigator->get_global_position()->alt + _navigator->get_takeoff_min_alt());

	} else {
		takeoff_alt = fmaxf(takeoff_alt, _navigator->get_home_position()->alt + _navigator->get_takeoff_min_alt());
	}

	return takeoff_alt;
}

void
Mission::heading_sp_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet =
		_navigator->get_position_setpoint_triplet();

	// Only update if current triplet is valid
	if (pos_sp_triplet->current.valid) {

		double point_from_latlon[2] = { _navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon
					      };
		double point_to_latlon[2] = { _navigator->get_global_position()->lat,
					      _navigator->get_global_position()->lon
					    };
		float yaw_offset = 0.0f;

		// Depending on ROI-mode, update heading
		switch (_navigator->get_vroi().mode) {
		case vehicle_roi_s::ROI_LOCATION: {
				// ROI is a fixed location. Vehicle needs to point towards that location
				point_to_latlon[0] = _navigator->get_vroi().lat;
				point_to_latlon[1] = _navigator->get_vroi().lon;
				// No yaw offset required
				yaw_offset = 0.0f;
				break;
			}

		case vehicle_roi_s::ROI_WPNEXT: {
				// ROI is current waypoint. Vehcile needs to point towards current waypoint
				point_to_latlon[0] = pos_sp_triplet->current.lat;
				point_to_latlon[1] = pos_sp_triplet->current.lon;
				// Add the gimbal's yaw offset
				yaw_offset = _navigator->get_vroi().yaw_offset;
				break;
			}

		case vehicle_roi_s::ROI_NONE:
		case vehicle_roi_s::ROI_WPINDEX:
		case vehicle_roi_s::ROI_TARGET:
		case vehicle_roi_s::ROI_ENUM_END:
		default: {
				return;
			}
		}

		// Get desired heading and update it.
		// However, only update if distance to desired heading is
		// larger than acceptance radius to prevent excessive yawing
		float d_current = get_distance_to_next_waypoint(point_from_latlon[0],
				  point_from_latlon[1], point_to_latlon[0], point_to_latlon[1]);

		if (d_current > _navigator->get_acceptance_radius()) {
			float yaw = matrix::wrap_pi(
					    get_bearing_to_next_waypoint(point_from_latlon[0],
							    point_from_latlon[1], point_to_latlon[0],
							    point_to_latlon[1]) + yaw_offset);

			_mission_item.yaw = yaw;
			pos_sp_triplet->current.yaw = _mission_item.yaw;
			pos_sp_triplet->current.yaw_valid = true;

		} else {
			if (!pos_sp_triplet->current.yaw_valid) {
				_mission_item.yaw = _navigator->get_local_position()->heading;
				pos_sp_triplet->current.yaw = _mission_item.yaw;
				pos_sp_triplet->current.yaw_valid = true;
			}
		}

		// we set yaw directly so we can run this in parallel to the FOH update
		publish_navigator_mission_item();
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
Mission::cruising_speed_sp_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	const float cruising_speed = _navigator->get_cruising_speed();

	/* Don't change setpoint if the current waypoint is not valid */
	if (!pos_sp_triplet->current.valid ||
	    fabsf(pos_sp_triplet->current.cruising_speed - cruising_speed) < FLT_EPSILON) {
		return;
	}

	pos_sp_triplet->current.cruising_speed = cruising_speed;

	publish_navigator_mission_item();
	_navigator->set_position_setpoint_triplet_updated();
}

void
Mission::do_abort_landing()
{
	// Abort FW landing, loiter above landing site in at least MIS_LND_ABRT_ALT

	if (_mission_item.nav_cmd != NAV_CMD_LAND) {
		return;
	}

	const float alt_landing = get_absolute_altitude_for_item(_mission_item);
	const float alt_sp = math::max(alt_landing + _navigator->get_landing_abort_min_alt(),
				       _navigator->get_global_position()->alt);

	// turn current landing waypoint into an indefinite loiter
	_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	_mission_item.altitude_is_relative = false;
	_mission_item.altitude = alt_sp;
	_mission_item.loiter_radius = _navigator->get_loiter_radius();
	_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
	_mission_item.autocontinue = false;
	_mission_item.origin = ORIGIN_ONBOARD;

	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &_navigator->get_position_setpoint_triplet()->current);

	// XXX: this is a hack to invalidate the "next" position setpoint for the fixed-wing position controller during
	// the landing abort hold. otherwise, the "next" setpoint would still register as a "LAND" point, and trigger
	// the early landing configuration (flaps and landing airspeed) during the hold.
	_navigator->get_position_setpoint_triplet()->next.lat = (double)NAN;
	_navigator->get_position_setpoint_triplet()->next.lon = (double)NAN;
	_navigator->get_position_setpoint_triplet()->next.alt = NAN;

	publish_navigator_mission_item(); // for logging
	_navigator->set_position_setpoint_triplet_updated();

	mavlink_log_info(_navigator->get_mavlink_log_pub(), "Holding at %d m above landing waypoint.\t",
			 (int)(alt_sp - alt_landing));
	events::send<float>(events::ID("mission_holding_above_landing"), events::Log::Info,
			    "Holding at {1:.0m_v} above landing waypoint", alt_sp - alt_landing);

	// reset mission index to start of landing
	if (_land_start_available) {
		_current_mission_index = get_land_start_index();

	} else {
		// move mission index back (landing approach point)
		_current_mission_index -= 1;
	}

	// send reposition cmd to get out of mission
	vehicle_command_s vcmd = {};

	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
	vcmd.param1 = -1;
	vcmd.param2 = 1;
	vcmd.param5 = _mission_item.lat;
	vcmd.param6 = _mission_item.lon;
	vcmd.param7 = alt_sp;

	_navigator->publish_vehicle_cmd(&vcmd);
}

bool
Mission::prepare_mission_items(struct mission_item_s *mission_item,
			       struct mission_item_s *next_position_mission_item, bool *has_next_position_item,
			       struct mission_item_s *after_next_position_mission_item, bool *has_after_next_position_item)
{
	*has_next_position_item = false;
	bool first_res = false;
	int offset = 1;

	if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
		offset = -1;
	}

	if (read_mission_item(0, mission_item)) {

		first_res = true;

		/* trying to find next position mission item */
		while (read_mission_item(offset, next_position_mission_item)) {
			if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
				offset--;

			} else {
				offset++;
			}

			if (item_contains_position(*next_position_mission_item)) {
				*has_next_position_item = true;
				break;
			}
		}

		if (_mission_execution_mode != mission_result_s::MISSION_EXECUTION_MODE_REVERSE &&
		    after_next_position_mission_item && has_after_next_position_item) {
			/* trying to find next next position mission item */
			while (read_mission_item(offset, after_next_position_mission_item)) {
				offset++;

				if (item_contains_position(*after_next_position_mission_item)) {
					*has_after_next_position_item = true;
					break;
				}
			}
		}
	}

	return first_res;
}

bool
Mission::read_mission_item(int offset, struct mission_item_s *mission_item)
{
	/* select mission */
	const int current_index = _current_mission_index;
	int index_to_read = current_index + offset;

	int *mission_index_ptr = (offset == 0) ? (int *) &_current_mission_index : &index_to_read;
	const dm_item_t dm_item = (dm_item_t)_mission.dataman_id;

	/* do not work on empty missions */
	if (_mission.count == 0) {
		return false;
	}

	/* Repeat this several times in case there are several DO JUMPS that we need to follow along, however, after
	 * 10 iterations we have to assume that the DO JUMPS are probably cycling and give up. */
	for (int i = 0; i < 10; i++) {
		if (*mission_index_ptr < 0 || *mission_index_ptr >= (int)_mission.count) {
			/* mission item index out of bounds - if they are equal, we just reached the end */
			if ((*mission_index_ptr != (int)_mission.count) && (*mission_index_ptr != -1)) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(),
						     "Mission item index out of bound, index: %d, max: %" PRIu16 ".\t",
						     *mission_index_ptr, _mission.count);
				events::send<uint16_t, uint16_t>(events::ID("mission_index_out_of_bound"), events::Log::Error,
								 "Mission item index out of bound, index: {1}, max: {2}", *mission_index_ptr, _mission.count);
			}

			return false;
		}

		const ssize_t len = sizeof(struct mission_item_s);

		/* read mission item to temp storage first to not overwrite current mission item if data damaged */
		struct mission_item_s mission_item_tmp;

		/* read mission item from datamanager */
		if (dm_read(dm_item, *mission_index_ptr, &mission_item_tmp, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Waypoint could not be read.\t");
			events::send<uint16_t>(events::ID("mission_failed_to_read_wp"), events::Log::Error,
					       "Waypoint {1} could not be read from storage", *mission_index_ptr);
			return false;
		}

		/* check for DO_JUMP item, and whether it hasn't not already been repeated enough times */
		if (mission_item_tmp.nav_cmd == NAV_CMD_DO_JUMP) {
			const bool execute_jumps = _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_NORMAL;

			/* do DO_JUMP as many times as requested if not in reverse mode */
			if ((mission_item_tmp.do_jump_current_count < mission_item_tmp.do_jump_repeat_count) && execute_jumps) {

				/* only raise the repeat count if this is for the current mission item
				 * but not for the read ahead mission item */
				if (offset == 0) {
					(mission_item_tmp.do_jump_current_count)++;

					/* save repeat count */
					if (dm_write(dm_item, *mission_index_ptr, &mission_item_tmp, len) != len) {
						/* not supposed to happen unless the datamanager can't access the dataman */
						mavlink_log_critical(_navigator->get_mavlink_log_pub(), "DO JUMP waypoint could not be written.\t");
						events::send(events::ID("mission_failed_to_write_do_jump"), events::Log::Error,
							     "DO JUMP waypoint could not be written");
						return false;
					}

					report_do_jump_mission_changed(*mission_index_ptr, mission_item_tmp.do_jump_repeat_count);
				}

				/* set new mission item index and repeat
				 * we don't have to validate here, if it's invalid, we should realize this later .*/
				*mission_index_ptr = mission_item_tmp.do_jump_mission_index;

			} else {
				if (offset == 0 && execute_jumps) {
					mavlink_log_info(_navigator->get_mavlink_log_pub(), "DO JUMP repetitions completed.\t");
					events::send(events::ID("mission_do_jump_rep_completed"), events::Log::Info,
						     "DO JUMP repetitions completed");
				}

				/* no more DO_JUMPS, therefore just try to continue with next mission item */
				if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
					(*mission_index_ptr)--;

				} else {
					(*mission_index_ptr)++;
				}
			}

		} else {
			/* if it's not a DO_JUMP, then we were successful */
			memcpy(mission_item, &mission_item_tmp, sizeof(struct mission_item_s));
			return true;
		}
	}

	/* we have given up, we don't want to cycle forever */
	mavlink_log_critical(_navigator->get_mavlink_log_pub(), "DO JUMP is cycling, giving up.\t");
	events::send(events::ID("mission_do_jump_cycle"), events::Log::Error, "DO JUMP is cycling, giving up");
	return false;
}

void
Mission::save_mission_state()
{
	mission_s mission_state = {};

	/* lock MISSION_STATE item */
	int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM_KEY_MISSION_STATE lock failed");
	}

	/* read current state */
	int read_res = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

	if (read_res == sizeof(mission_s)) {
		/* data read successfully, check dataman ID and items count */
		if (mission_state.dataman_id == _mission.dataman_id && mission_state.count == _mission.count) {
			/* navigator may modify only sequence, write modified state only if it changed */
			if (mission_state.current_seq != _current_mission_index) {
				mission_state.current_seq = _current_mission_index;
				mission_state.timestamp = hrt_absolute_time();

				if (dm_write(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s)) != sizeof(mission_s)) {

					PX4_ERR("Can't save mission state");
				}
			}
		}

	} else {
		/* invalid data, this must not happen and indicates error in mission publisher */
		mission_state.timestamp = hrt_absolute_time();
		mission_state.dataman_id = _mission.dataman_id;
		mission_state.count = _mission.count;
		mission_state.current_seq = _current_mission_index;

		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Invalid mission state.\t");
		/* EVENT
		 * @description No mission or storage failure
		 */
		events::send(events::ID("mission_invalid_mission_state"), events::Log::Error, "Invalid mission state");

		/* write modified state only if changed */
		if (dm_write(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s)) != sizeof(mission_s)) {

			PX4_ERR("Can't save mission state");
		}
	}

	/* unlock MISSION_STATE item */
	if (dm_lock_ret == 0) {
		dm_unlock(DM_KEY_MISSION_STATE);
	}
}

void
Mission::report_do_jump_mission_changed(int index, int do_jumps_remaining)
{
	/* inform about the change */
	_navigator->get_mission_result()->item_do_jump_changed = true;
	_navigator->get_mission_result()->item_changed_index = index;
	_navigator->get_mission_result()->item_do_jump_remaining = do_jumps_remaining;

	_navigator->set_mission_result_updated();
}

void
Mission::set_mission_item_reached()
{
	_navigator->get_mission_result()->seq_reached = _current_mission_index;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();
}

void
Mission::set_current_mission_item()
{
	_navigator->get_mission_result()->finished = false;
	_navigator->get_mission_result()->seq_current = _current_mission_index;

	_navigator->set_mission_result_updated();

	save_mission_state();
}

void
Mission::check_mission_valid(bool force)
{
	if ((!_home_inited && _navigator->home_global_position_valid()) || force) {

		MissionFeasibilityChecker _missionFeasibilityChecker(_navigator);

		_navigator->get_mission_result()->valid =
			_missionFeasibilityChecker.checkMissionFeasible(_mission);

		_navigator->get_mission_result()->seq_total = _mission.count;
		_navigator->increment_mission_instance_count();
		_navigator->set_mission_result_updated();
		_home_inited = _navigator->home_global_position_valid();

		// find and store landing start marker (if available)
		find_mission_land_start();
	}
}

void
Mission::reset_mission(struct mission_s &mission)
{
	dm_lock(DM_KEY_MISSION_STATE);

	if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) == sizeof(mission_s)) {
		if (mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 || mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1) {
			/* set current item to 0 */
			mission.current_seq = 0;

			/* reset jump counters */
			if (mission.count > 0) {
				const dm_item_t dm_current = (dm_item_t)mission.dataman_id;

				for (unsigned index = 0; index < mission.count; index++) {
					struct mission_item_s item;
					const ssize_t len = sizeof(struct mission_item_s);

					if (dm_read(dm_current, index, &item, len) != len) {
						PX4_WARN("could not read mission item during reset");
						break;
					}

					if (item.nav_cmd == NAV_CMD_DO_JUMP) {
						item.do_jump_current_count = 0;

						if (dm_write(dm_current, index, &item, len) != len) {
							PX4_WARN("could not save mission item during reset");
							break;
						}
					}
				}
			}

		} else {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Could not read mission.\t");
			events::send(events::ID("mission_cannot_read_mission"), events::Log::Error, "Could not read mission");

			/* initialize mission state in dataman */
			mission.timestamp = hrt_absolute_time();
			mission.dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
			mission.count = 0;
			mission.current_seq = 0;
		}

		dm_write(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s));
	}

	dm_unlock(DM_KEY_MISSION_STATE);
}

bool
Mission::need_to_reset_mission()
{
	/* reset mission state when disarmed */
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED && _need_mission_reset) {
		_need_mission_reset = false;
		return true;
	}

	return false;
}

int32_t
Mission::index_closest_mission_item() const
{
	int32_t min_dist_index(0);
	float min_dist(FLT_MAX), dist_xy(FLT_MAX), dist_z(FLT_MAX);

	dm_item_t dm_current = (dm_item_t)(_mission.dataman_id);

	for (size_t i = 0; i < _mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(missionitem);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			PX4_ERR("dataman read failure");
			break;
		}

		if (item_contains_position(missionitem)) {
			// do not consider land waypoints for a fw
			if (!((missionitem.nav_cmd == NAV_CMD_LAND) &&
			      (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) &&
			      (!_navigator->get_vstatus()->is_vtol))) {
				float dist = get_distance_to_point_global_wgs84(missionitem.lat, missionitem.lon,
						get_absolute_altitude_for_item(missionitem),
						_navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon,
						_navigator->get_global_position()->alt,
						&dist_xy, &dist_z);

				if (dist < min_dist) {
					min_dist = dist;
					min_dist_index = i;
				}
			}
		}
	}

	// for mission reverse also consider the home position
	if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
		float dist = get_distance_to_point_global_wgs84(
				     _navigator->get_home_position()->lat,
				     _navigator->get_home_position()->lon,
				     _navigator->get_home_position()->alt,
				     _navigator->get_global_position()->lat,
				     _navigator->get_global_position()->lon,
				     _navigator->get_global_position()->alt,
				     &dist_xy, &dist_z);

		if (dist < min_dist) {
			min_dist = dist;
			min_dist_index = -1;
		}
	}

	return min_dist_index;
}

bool Mission::position_setpoint_equal(const position_setpoint_s *p1, const position_setpoint_s *p2) const
{
	return ((p1->valid == p2->valid) &&
		(p1->type == p2->type) &&
		(fabsf(p1->vx - p2->vx) < FLT_EPSILON) &&
		(fabsf(p1->vy - p2->vy) < FLT_EPSILON) &&
		(fabsf(p1->vz - p2->vz) < FLT_EPSILON) &&
		(fabs(p1->lat - p2->lat) < DBL_EPSILON) &&
		(fabs(p1->lon - p2->lon) < DBL_EPSILON) &&
		(fabsf(p1->alt - p2->alt) < FLT_EPSILON) &&
		((fabsf(p1->yaw - p2->yaw) < FLT_EPSILON) || (!PX4_ISFINITE(p1->yaw) && !PX4_ISFINITE(p2->yaw))) &&
		(p1->yaw_valid == p2->yaw_valid) &&
		(fabsf(p1->yawspeed - p2->yawspeed) < FLT_EPSILON) &&
		(p1->yawspeed_valid == p2->yawspeed_valid) &&
		(fabsf(p1->loiter_radius - p2->loiter_radius) < FLT_EPSILON) &&
		(p1->loiter_direction_counter_clockwise == p2->loiter_direction_counter_clockwise) &&
		(fabsf(p1->acceptance_radius - p2->acceptance_radius) < FLT_EPSILON) &&
		(fabsf(p1->cruising_speed - p2->cruising_speed) < FLT_EPSILON) &&
		((fabsf(p1->cruising_throttle - p2->cruising_throttle) < FLT_EPSILON) || (!PX4_ISFINITE(p1->cruising_throttle)
				&& !PX4_ISFINITE(p2->cruising_throttle))));

}

void Mission::publish_navigator_mission_item()
{
	navigator_mission_item_s navigator_mission_item{};

	navigator_mission_item.instance_count = _navigator->mission_instance_count();
	navigator_mission_item.sequence_current = _current_mission_index;
	navigator_mission_item.nav_cmd = _mission_item.nav_cmd;
	navigator_mission_item.latitude = _mission_item.lat;
	navigator_mission_item.longitude = _mission_item.lon;
	navigator_mission_item.altitude = _mission_item.altitude;

	navigator_mission_item.time_inside = get_time_inside(_mission_item);
	navigator_mission_item.acceptance_radius = _mission_item.acceptance_radius;
	navigator_mission_item.loiter_radius = _mission_item.loiter_radius;
	navigator_mission_item.yaw = _mission_item.yaw;

	navigator_mission_item.frame = _mission_item.frame;
	navigator_mission_item.frame = _mission_item.origin;

	navigator_mission_item.loiter_exit_xtrack = _mission_item.loiter_exit_xtrack;
	navigator_mission_item.force_heading = _mission_item.force_heading;
	navigator_mission_item.altitude_is_relative = _mission_item.altitude_is_relative;
	navigator_mission_item.autocontinue = _mission_item.autocontinue;
	navigator_mission_item.vtol_back_transition = _mission_item.vtol_back_transition;

	navigator_mission_item.timestamp = hrt_absolute_time();

	_navigator_mission_item_pub.publish(navigator_mission_item);
=======
>>>>>>> 64f28c4c076f8bead474b19c18c3a9a6dbcaccbf
}
