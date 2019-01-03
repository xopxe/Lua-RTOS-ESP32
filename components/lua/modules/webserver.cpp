#include "sdkconfig.h"

#if CONFIG_LUA_RTOS_LUA_USE_WEBSERVER

#include "lua.hpp"

#include <HttpServer.h>

extern "C"{

#include "modules.h"
//#include "error.h"

HttpServer httpServer;

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
    lua_pushboolean(L, true);
	return 1;
}


static const luaL_Reg webserver[] = {
    {"init", lwebserver_init},
    {"release", lwebserver_release},
    {NULL, NULL}
};

LUALIB_API int luaopen_webserver( lua_State *L ) {
    //luaL_register(L,"vl53l0x", vl53l0x_map);
    luaL_newlib(L, webserver);
	return 1;
}

MODULE_REGISTER_RAM(WEBSERVER, webserver, luaopen_webserver, 1);


}

#endif
