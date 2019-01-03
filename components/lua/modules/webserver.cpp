#include "sdkconfig.h"

#if CONFIG_LUA_RTOS_LUA_USE_WEBSERVER

#include "lua.hpp"

#include <HttpServer.h>

extern "C"{

#include "modules.h"
#include <esp_log.h>
//#include "error.h"

HttpServer httpServer;

int websocket_lua_callback = LUA_NOREF;

static void helloWorldHandler(HttpRequest* pRequest, HttpResponse* pResponse) {
    printf(">>helloWorldHandler\r\n");
    pResponse->setStatus(HttpResponse::HTTP_STATUS_OK, "OK");
    pResponse->addHeader(HttpRequest::HTTP_HEADER_CONTENT_TYPE, "text/plain");
    pResponse->sendData("Hello back");
    pResponse->close();
}    

static int lwebserver_init (lua_State *L) {

    httpServer.setRootPath(CONFIG_LUA_RTOS_WEBSERVER_ROOT);

    httpServer.addPathHandler(
        HttpRequest::HTTP_METHOD_GET,
        "/helloWorld",
        helloWorldHandler);

    httpServer.start(CONFIG_LUA_RTOS_WEBSERVER_PORT);

    lua_pushboolean(L, true);
	return 1;
}

static int lwebserver_release (lua_State *L) {
	httpServer.stop();
    lua_pushboolean(L, true);
	return 1;
}

class MyWebSocketHandler : public WebSocketHandler {
public:
	void onClose() {
		ESP_LOGD("MyWebSocketHandler", ">> onClose");
		ESP_LOGD("MyWebSocketHandler", "<< onClose");
	} // onClose
	void onMessage(WebSocketInputStreambuf* pWebSocketInputStreambuf, WebSocket* pWebSocket) {
		ESP_LOGD("MyWebSocketHandler", ">> onMessage");
		ESP_LOGD("MyWebSocketHandler", "<< onMessage");
	} // onData
};
MyWebSocketHandler myWebSocketHandler;


static void websocketAcceptHandler(HttpRequest* pRequest, HttpResponse* pResponse) {
    printf(">>websocketdHandler\r\n"); 
	if (pRequest->isWebsocket()) {
	    printf(">>websocketdHandler is WebSocket\r\n"); 
		WebSocket *webSocket = pRequest->getWebSocket();

		webSocket->setHandler(&myWebSocketHandler);
	}
}


static int lwebserver_ws_register (lua_State *L) {
	const char * path = lua_tolstring(L, 1, NULL);

	if (lua_isfunction(L, 2)) {
		luaL_checktype(L, 2, LUA_TFUNCTION);
		lua_pushvalue(L, 2);
		websocket_lua_callback = luaL_ref(L, LUA_REGISTRYINDEX);
	} else {
		websocket_lua_callback = LUA_NOREF;
	}

    httpServer.addPathHandler(
        HttpRequest::HTTP_METHOD_GET,
        path,
        websocketAcceptHandler);

    lua_pushboolean(L, true);
	return 1;
}

static const luaL_Reg webserver[] = {
    {"init", lwebserver_init},
    {"release", lwebserver_release},
    {"ws_register", lwebserver_ws_register},
    {NULL, NULL}
};

LUALIB_API int luaopen_webserver( lua_State *L ) {

	//esp_log_level_set("HttpServerTask", ESP_LOG_DEBUG);


    //luaL_register(L,"vl53l0x", vl53l0x_map);
    luaL_newlib(L, webserver);
	return 1;
}

MODULE_REGISTER_RAM(WEBSERVER, webserver, luaopen_webserver, 1);


}

#endif
