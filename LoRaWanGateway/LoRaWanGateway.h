#ifndef __LoRaWanGateway__
#define __LoRaWanGateway__

#include <DebugM.h>

#include <Application.h>
#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>
#include <Login.h>
#include <NetworkNode.h>
#include <SystemClock.h>
#include <HTTPServer.h>
#include <WebSocketsServer.h>
#include <WIFI.h>
#include <RFM.h>
#include <System.h>
#include <WAN.h>


class LoRaWanGateway : public Application, public NetworkNode {
	public:
	WIFI* wifi = NULL;
	WAN* wan = NULL;
	RFM* rfm = NULL;
	System* system = NULL;

	uint64_t lping = 0ull;
	uint32_t iping = 15ul * 1000ul;

	LoRaWanGateway();
	virtual ~LoRaWanGateway();
	void loop();
	void setup();
};

#endif
