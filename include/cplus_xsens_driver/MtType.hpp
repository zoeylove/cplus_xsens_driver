#ifndef _MTTYPE_H__
#define _MTTYPE_H__


namespace xsens_imu {

	enum GroupId{
		kTimestamp = 0x10,
		kOrientationData = 0x20,
		kAcceleration = 0x40,
		kAngularVelocity = 0x80,
		kStatus = 0xe0
	};
 
	const u_int8_t kMtData2 = 0x36;



}



#endif // !_MTTYPE_H__

