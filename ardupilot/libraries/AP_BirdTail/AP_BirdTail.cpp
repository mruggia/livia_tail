#include "AP_BirdTail.h"

#include <SRV_Channel/SRV_Channel.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;
AP_BirdTail *AP_BirdTail::_singleton;
const AP_Param::GroupInfo AP_BirdTail::var_info[] = { AP_GROUPEND };


// input ranges
const float input_S_min = 1e-6,      input_S_max = 2.0/3.0*M_PI;// min/max feasible tail spread angles
const float input_Y_min = -M_PI/6.0, input_Y_max = M_PI/6.0;	// min/max feasible tail yaw angles
const float input_P_min = -M_PI/4.0, input_P_max = M_PI/4.0;	// min/max feasible tail pitch angles
const float input_R_min = -M_PI/4.0, input_R_max = M_PI/4.0;	// min/max feasible tail roll angles

// input settings
const int16_t input_mode_channel = 6;			// rc channel for switching tail control modes
const float   input_flight_P_scale = 0.8;		// pitch scaling factor for flight
const float   input_flight_P_trim = 0.0;		// pitch trim [-1,1]
const float   input_flight_spread = M_PI/2.0;	// spread for flight

// servo calibration
const float servo_RB_offset = -0.15;	// right-bottom servo center offset [-1,1]
const float servo_LB_offset = -0.15;	// left-bottom servo center offset [-1,1]
const float servo_RT_offset	= -0.33;	// right-top servo center offset [-1,1]
const float servo_LT_offset = -0.20;	// left-top servo center offset [-1,1]
const float servo_out_per_rad = 2.0/(145.0/180.0*M_PI);	// servo output per radian angle

// tail geometry
const float l1 = 14.0;			// tail to pivot distance
const float l2 = l1*M_PI/2.0;	// tail nominal half width
const float l3 = 2.6;			// tail lever outside offset
const float l4 = l2-l1+l3;		// tail lever horizontal length
const float l5 = 7.6;			// tail lever vertical half length
const float l6 = 22.0;			// servo-tail link bar
const float l7 = 16.2;			// servo arm length
const float l8 = 8.0/2.0;		// servo center height
const float l9 = 25.6;			// servo width
const float l10= 10.0;			// distance servo axis from pivot

// status reports to ground station
const bool _status_report_enable = true;	// flag enabling status reports to ground station
const uint32_t _status_period_ms = 2000;	// period between each status report in miliseconds
uint32_t _status_last_ms = 0;				// time since last status report


// constructor
AP_BirdTail::AP_BirdTail(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_BirdTail must be singleton");
    }
    _singleton = this;
}

// update and report, called from main loop
void AP_BirdTail::update(void)
{
	// get input angles (spread, yaw, pitch, roll)
	float R[4] = { 1e-6, 0.0, 0.0, 0.0 };
	
	// flight with pitch only
	const int16_t input_mode = RC_Channels::get_radio_in(input_mode_channel);
	if ( input_mode < 1250.0 ) {
		R[0] = input_flight_spread;
		R[1] = 0.0;
		R[2] = -linear_interpolate(input_P_min,input_P_max, SRV_Channels::get_output_norm(SRV_Channel::k_elevator)+input_flight_P_trim, -1.0,1.0) * input_flight_P_scale;
		R[3] = 0.0;
	}
	// flight with full contorl (disabled for now)
	else if ( input_mode < 1750.0 ) {
		R[0] = M_PI/4.0;
		R[1] = 0.0;
		R[2] = 0.0;
		R[3] = 0.0;
	}
	// ground demo
	else {
		R[0] = linear_interpolate(input_S_min,input_S_max, RC_Channels::get_radio_in(2), 1000.0,2000.0);
		R[1] = linear_interpolate(input_Y_min,input_Y_max, RC_Channels::get_radio_in(3), 1000.0,2000.0);
		R[2] = linear_interpolate(input_P_min,input_P_max, RC_Channels::get_radio_in(1), 1000.0,2000.0);
		R[3] = linear_interpolate(input_R_min,input_R_max, RC_Channels::get_radio_in(0), 1000.0,2000.0);
	}
	// constrain inputs
	R[0] = constrain_value( R[0], input_S_min,input_S_max);
	R[1] = constrain_value( R[1], input_Y_min,input_Y_max);
	R[2] = constrain_value( R[2], input_P_min,input_P_max);
	R[3] = constrain_value( R[3], input_R_min,input_R_max);
	
	// calc arc parameters
    const float r_arc = 2.0*l2/R[0];
    const float a_arc = R[0]/2.0;
	// calc tail points with spread only
	const float cs = cosf(a_arc), ss = sinf(a_arc);
	const float p6RBs[3]  = { 
		r_arc-r_arc*cs-l1 + l3*ss - l4*cs, 
		r_arc*ss          + l3*cs + l4*ss, 
		l5
	};
	const float p6LBs[3] = { +p6RBs[0], -p6RBs[1], +p6RBs[2] };
	const float p6RTs[3] = { +p6RBs[0], +p6RBs[1], -p6RBs[2] };
	const float p6LTs[3] = { +p6RBs[0], -p6RBs[1], -p6RBs[2] };
	// calc rotation matrix
	const float cz = cosf(R[1]), sz = sinf(R[1]);
	const float cy = cosf(R[2]), sy = sinf(R[2]);
	const float cx = cosf(R[3]), sx = sinf(R[3]);
	const float Rot[3][3] = {
		{ cy*cz, cz*sx*sy-cx*sz, sx*sz+cx*cz*sy },
		{ cy*sz, cx*cz+sx*sy*sz, cx*sy*sz-cz*sx },
		{ -sy,   cy*sx,          cx*cy          }
	};
	// calc tail points with spread-roll-pitch-yaw
	const float p6RB[3] = { Rot[0][0]*p6RBs[0]+Rot[0][1]*p6RBs[1]+Rot[0][2]*p6RBs[2], Rot[1][0]*p6RBs[0]+Rot[1][1]*p6RBs[1]+Rot[1][2]*p6RBs[2], Rot[2][0]*p6RBs[0]+Rot[2][1]*p6RBs[1]+Rot[2][2]*p6RBs[2] };
	const float p6LB[3] = { Rot[0][0]*p6LBs[0]+Rot[0][1]*p6LBs[1]+Rot[0][2]*p6LBs[2], Rot[1][0]*p6LBs[0]+Rot[1][1]*p6LBs[1]+Rot[1][2]*p6LBs[2], Rot[2][0]*p6LBs[0]+Rot[2][1]*p6LBs[1]+Rot[2][2]*p6LBs[2] };
	const float p6RT[3] = { Rot[0][0]*p6RTs[0]+Rot[0][1]*p6RTs[1]+Rot[0][2]*p6RTs[2], Rot[1][0]*p6RTs[0]+Rot[1][1]*p6RTs[1]+Rot[1][2]*p6RTs[2], Rot[2][0]*p6RTs[0]+Rot[2][1]*p6RTs[1]+Rot[2][2]*p6RTs[2] };
	const float p6LT[3] = { Rot[0][0]*p6LTs[0]+Rot[0][1]*p6LTs[1]+Rot[0][2]*p6LTs[2], Rot[1][0]*p6LTs[0]+Rot[1][1]*p6LTs[1]+Rot[1][2]*p6LTs[2], Rot[2][0]*p6LTs[0]+Rot[2][1]*p6LTs[1]+Rot[2][2]*p6LTs[2] };
	// calc servo points
	const float p8RB[3] = { +l10, +l9, +l8 };
	const float p8LB[3] = { +l10, -l9, +l8 };
	const float p8RT[3] = { +l10, +l9, -l8 };
	const float p8LT[3] = { +l10, -l9, -l8 };
	// calc intermediate triangle lengths
	const float nRB = sqrtf( (p8RB[0]-p6RB[0])*(p8RB[0]-p6RB[0]) + (p8RB[2]-p6RB[2])*(p8RB[2]-p6RB[2]) );
	const float nLB = sqrtf( (p8LB[0]-p6LB[0])*(p8LB[0]-p6LB[0]) + (p8LB[2]-p6LB[2])*(p8LB[2]-p6LB[2]) );
	const float nRT = sqrtf( (p8RT[0]-p6RT[0])*(p8RT[0]-p6RT[0]) + (p8RT[2]-p6RT[2])*(p8RT[2]-p6RT[2]) );
	const float nLT = sqrtf( (p8LT[0]-p6LT[0])*(p8LT[0]-p6LT[0]) + (p8LT[2]-p6LT[2])*(p8LT[2]-p6LT[2]) );
	const float mRB = sqrtf( l6*l6 - (p8RB[1]-p6RB[1])*(p8RB[1]-p6RB[1]) );
	const float mLB = sqrtf( l6*l6 - (p8LB[1]-p6LB[1])*(p8LB[1]-p6LB[1]) );
	const float mRT = sqrtf( l6*l6 - (p8RT[1]-p6RT[1])*(p8RT[1]-p6RT[1]) );
	const float mLT = sqrtf( l6*l6 - (p8LT[1]-p6LT[1])*(p8LT[1]-p6LT[1]) );
	// calc intermediate triangle angles
	const float q1RB = acosf( (l7*l7+nRB*nRB-mRB*mRB)/(2*l7*nRB) );
	const float q1LB = acosf( (l7*l7+nLB*nLB-mLB*mLB)/(2*l7*nLB) );
	const float q1RT = acosf( (l7*l7+nRT*nRT-mRT*mRT)/(2*l7*nRT) );
	const float q1LT = acosf( (l7*l7+nLT*nLT-mLT*mLT)/(2*l7*nLT) );
	if ( isnan(q1RB) || isnan(q1LB) || isnan(q1RT) || isnan(q1LT) ) { return; }
	const float q2RB = M_PI - atan2f( p6RB[2]-p8RB[2], p6RB[0]-p8RB[0] );
	const float q2LB = M_PI - atan2f( p6LB[2]-p8LB[2], p6LB[0]-p8LB[0] );
	const float q2RT = M_PI + atan2f( p6RT[2]-p8RT[2], p6RT[0]-p8RT[0] );
	const float q2LT = M_PI + atan2f( p6LT[2]-p8LT[2], p6LT[0]-p8LT[0] );
	// calc servo angles
	const float qRB = constrain_value( wrap_PI( M_PI/2.0 - q1RB - q2RB ), -M_PI/3.0,M_PI/2.0 );
	const float qLB = constrain_value( wrap_PI( M_PI/2.0 - q1LB - q2LB ), -M_PI/3.0,M_PI/2.0 );
	const float qRT = constrain_value( wrap_PI( M_PI/2.0 - q1RT - q2RT ), -M_PI/3.0,M_PI/2.0 );
	const float qLT = constrain_value( wrap_PI( M_PI/2.0 - q1LT - q2LT ), -M_PI/3.0,M_PI/2.0 );
	
	// set servo angles
	SRV_Channels::set_output_norm(SRV_Channel::k_scripting1, -1.0*( qRB * servo_out_per_rad + servo_RB_offset) );
	SRV_Channels::set_output_norm(SRV_Channel::k_scripting2, +1.0*( qLB * servo_out_per_rad + servo_LB_offset) );
	SRV_Channels::set_output_norm(SRV_Channel::k_scripting3, +1.0*( qRT * servo_out_per_rad + servo_RT_offset) );
	SRV_Channels::set_output_norm(SRV_Channel::k_scripting4, -1.0*( qLT * servo_out_per_rad + servo_LT_offset) );
	
	// send status to ground station
    if (_status_report_enable && (AP_HAL::millis() - _status_last_ms) > _status_period_ms) {
        _status_last_ms = AP_HAL::millis();
		GCS_SEND_TEXT( MAV_SEVERITY_INFO, "AP_BirdTail:\nR = [ %.2f, %.2f, %.2f, %.2f ]\nU = [ %.2f, %.2f, %.2f, %.2f ]\n", R[0],R[1],R[2],R[3], qRB,qLB,qRT,qLT);
    }

}

namespace AP {
	AP_BirdTail &birdtail()	{ return *AP_BirdTail::get_singleton(); }
}