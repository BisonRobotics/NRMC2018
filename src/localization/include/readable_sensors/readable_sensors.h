#ifndef READABLE_SENSORS_H
#define READABLE_SENSORS_H

class ReadableSensors {
	public:
		enum class ReadStatus {READ_FAILED, READ_SUCCESS}; //this requires cpp11
		virtual ReadStatus receiveData()=0;

};

#endif