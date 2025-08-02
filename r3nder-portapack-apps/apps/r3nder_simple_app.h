#ifndef R3NDER_SIMPLE_APP_H
#define R3NDER_SIMPLE_APP_H

#include "app.h"
#include "ui/ui.h"
#include "hackrf/receiver.h"
#include "hackrf/transmitter.h"

#define SIMPLE_R3NDER_APP(APP_CLASS) \
class APP_CLASS : public App { \
public: \
    void init() override { ui.title(#APP_CLASS " - Init"); } \
    void run() override { ui.label(#APP_CLASS " - Running"); } \
    void stop() override { ui.label(#APP_CLASS " - Stopped"); } \
}; \
APP_FACTORY(APP_CLASS);

#endif // R3NDER_SIMPLE_APP_H
