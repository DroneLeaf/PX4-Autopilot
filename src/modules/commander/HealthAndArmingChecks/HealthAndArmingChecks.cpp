/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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

#include "HealthAndArmingChecks.hpp"

HealthAndArmingChecks::HealthAndArmingChecks(ModuleParams *parent, vehicle_status_s &status)
	: ModuleParams(parent),
	  _context(status)
{
	// Initialize mode requirements to invalid
	_failsafe_flags.angular_velocity_invalid = true;
	_failsafe_flags.attitude_invalid = true;
	_failsafe_flags.local_altitude_invalid = true;
	_failsafe_flags.local_position_invalid = true;
	_failsafe_flags.local_position_invalid_relaxed = true;
	_failsafe_flags.local_velocity_invalid = true;
	_failsafe_flags.global_position_invalid = true;
	_failsafe_flags.auto_mission_missing = true;
	_failsafe_flags.offboard_control_signal_lost = true;
	_failsafe_flags.home_position_invalid = true;
}

events::Log HealthAndArmingChecks::map_ext_to_log(uint8_t ext)
{
    switch (ext & 0x0F) {
    case 0: return events::Log::Emergency;
    case 1: return events::Log::Alert;
    case 2: return events::Log::Critical;
    case 3: return events::Log::Error;
    case 4: return events::Log::Warning;
    case 5: return events::Log::Notice;
    case 6: return events::Log::Info;
    default: return events::Log::Debug;
    }
}

// avoid the "unmatched events::ID(" assertion by breaking the literal pattern
#define LEAF_EVID(name) events::ID/**/(name)
void HealthAndArmingChecks::checkLeafHealth(Report &reporter)
{


    // Consider it "active" if a message arrived recently (e.g., in last 2 s)

        const events::Log sev = HealthAndArmingChecks::map_ext_to_log(_leaf_last.ext_severity);

	switch (_leaf_last.remote_event_id)
	{
		case LEAF_EVID("optitrack_stream_lost"):
			// PX4_INFO("Leaf optitrack_stream_lost %" PRIu8,_leaf_last.arguments[0]);
			leafEventSet[0]=_leaf_last.arguments[0];
			break;

		case LEAF_EVID("misconfigured_esc"):
			// PX4_INFO("Leaf misconfigured_esc %" PRIu8,_leaf_last.arguments[0]);
			leafEventSet[1]=_leaf_last.arguments[0];
			break;

		case LEAF_EVID("misconfigured_geometry"):
			// PX4_INFO("Leaf misconfigured_geometry %" PRIu8,_leaf_last.arguments[0]);
			leafEventSet[2]=_leaf_last.arguments[0];
			break;

		case LEAF_EVID("misconfigured_rc"):
			// PX4_INFO("Leaf misconfigured_rc %" PRIu8,_leaf_last.arguments[0]);
			leafEventSet[3]=_leaf_last.arguments[0];
			break;

		case LEAF_EVID("insufficient_memory"):
			// PX4_INFO("Leaf insufficient_memory %" PRIu8,_leaf_last.arguments[0]);
			leafEventSet[4]=_leaf_last.arguments[0];
			break;
		default:
			break;

	}

	if (leafEventSet[0])
	{
		/* EVENT
		* @description
		* Lost OptiTrack Stream. See <a href="https://fly.droneleaf.io/">OptiTrack Website</a>
		*/
		reporter.armingCheckFailure(
		NavModes::All,                   // affect all nav modes; tailor if needed
		health_component_t::leaf,        // your custom component
		events::ID("optitrack_stream_lost"),
		sev,
		"OptiTrack Stream Lost"
		);
	}
	if (leafEventSet[1])
	{
		/* EVENT
		* @description
		* Motors are not configured properly. See <a href="https://fly.droneleaf.io/">ESCs Workflow</a>
		*/
		reporter.armingCheckFailure(
		NavModes::All,                   // affect all nav modes; tailor if needed
		health_component_t::leaf,        // your custom component
		events::ID("misconfigured_esc"),
		sev,
		"Motors not configured properly"
		);
	}

	if (leafEventSet[2])
	{
		/* EVENT
		* @description
		* Drone geometry is not configured properly. See <a href="https://fly.droneleaf.io/">Geometry Workflow</a>
		*/
		reporter.armingCheckFailure(
		NavModes::All,                   // affect all nav modes; tailor if needed
		health_component_t::leaf,        // your custom component
		events::ID("misconfigured_geometry"),
		sev,
		"Geometry not configured properly"
		);
	}

	if (leafEventSet[3])
	{
		/* EVENT
		* @description
		* Drone RC is not configured properly. See <a href="https://fly.droneleaf.io/">RC Workflow</a>
		*/
		reporter.armingCheckFailure(
		NavModes::All,                   // affect all nav modes; tailor if needed
		health_component_t::leaf,        // your custom component
		events::ID("misconfigured_rc"),
		sev,
		"RC not configured properly"
		);
	}

	if (leafEventSet[4])
	{
		/* EVENT
		* @description
		* Insufficient LeafFC memory. <a href="https://fly.droneleaf.io/">Clear existing logs</a>
		*/
		reporter.armingCheckFailure(
		NavModes::All,                   // affect all nav modes; tailor if needed
		health_component_t::leaf,        // your custom component
		events::ID("insufficient_memory"),
		sev,
		"Insufficient LeafFC memory"
		);
	}

	// Drain any new messages; keep the most recent
	leaf_health_events_s le{};
	//     while (_leaf_health_sub.update(&le)) {
	//         _leaf_last = le;
	//         _leaf_last_ts = le.timestamp;
	//     }
	if (!_leaf_health_sub.update(&le)){
		return;
	}
	_leaf_last=le;
}



bool HealthAndArmingChecks::update(bool force_reporting)
{
	_reporter.reset();

	_reporter.prepare(_context.status().vehicle_type);

	if (!_context.isArmed()){
		checkLeafHealth(_reporter);
	}

	for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
		if (!_checks[i]) {
			break;
		}

		_checks[i]->checkAndReport(_context, _reporter);
	}

	const bool results_changed = _reporter.finalize();
	const bool reported = _reporter.report(_context.isArmed(), force_reporting);

	if (reported) {

		// LEGACY start
		// Run the checks again, this time with the mavlink publication set.
		// We don't expect any change, and rate limitation would prevent the events from being reported again,
		// so we only report mavlink_log_*.
		_reporter._mavlink_log_pub = &_mavlink_log_pub;
		_reporter.reset();

		_reporter.prepare(_context.status().vehicle_type);

		if (!_context.isArmed()){
			checkLeafHealth(_reporter);
		}

		for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
			if (!_checks[i]) {
				break;
			}

			_checks[i]->checkAndReport(_context, _reporter);
		}

		_reporter.finalize();
		_reporter.report(_context.isArmed(), false);
		_reporter._mavlink_log_pub = nullptr;
		// LEGACY end

		health_report_s health_report;
		_reporter.getHealthReport(health_report);
		health_report.timestamp = hrt_absolute_time();
		_health_report_pub.publish(health_report);
	}

	// Check if we need to publish the failsafe flags
	const hrt_abstime now = hrt_absolute_time();

	if ((now > _failsafe_flags.timestamp + 500_ms) || results_changed) {
		_failsafe_flags.timestamp = hrt_absolute_time();
		_failsafe_flags_pub.publish(_failsafe_flags);
	}

	return reported;
}

void HealthAndArmingChecks::updateParams()
{
	for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
		if (!_checks[i]) {
			break;
		}

		_checks[i]->updateParams();
	}
}
