#include <PLS200/libpcan.h>
#include <PLS200/common.c>
#include <PLS200/parser.cpp>

namespace PowerCube_Driver
{

  ////////////////////////////////////////////////////////////////////////////////
  class PowerCube
  {
    public:
      char* texto;
      int Aux;

      int nExtended;

      PowerCube();
  };
}
