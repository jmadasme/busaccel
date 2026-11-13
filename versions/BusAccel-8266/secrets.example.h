// secrets.example.h — Plantilla de credenciales
// Copia este archivo como secrets.h y rellena con tus valores reales.
// secrets.h NO se versiona (esta en .gitignore).
#ifndef SECRETS_H
#define SECRETS_H

// STA: red Nv2 (opcional; si no esta disponible, el hotspot AP sigue activo)
#define WIFI_SSID     "TU_RED_WIFI"
#define WIFI_PASSWORD "TU_CLAVE_WIFI"

// AP: hotspot propio que la ESP8266 crea SIEMPRE (min 8 caracteres)
#define AP_SSID       "BusAccel-8266"
#define AP_PASSWORD   "TU_CLAVE_AP_8MAS"

#endif