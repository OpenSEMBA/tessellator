#pragma once

#include "MesherBaseOptions.h"

namespace meshlib::meshers {

class StaircaseMesherOptions : public MesherBaseOptions {
public:
    bool compress = false;
};

}
