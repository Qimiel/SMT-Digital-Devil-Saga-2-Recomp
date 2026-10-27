#pragma once

// Runs VU1 microprograms through their C++ translations (tools/vu1_translate.py)
// instead of the interpreter. PS2X_VU1_NATIVE=0 turns it off; PS2X_VU1_VERIFY=1
// runs each program both ways and reports differences.
namespace vu1rec
{
    void install();
}
