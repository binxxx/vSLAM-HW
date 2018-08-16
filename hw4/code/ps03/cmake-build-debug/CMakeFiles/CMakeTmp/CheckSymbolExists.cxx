/* */
#include <QtCore/qglobal.h>

int main(int argc, char** argv)
{
  (void)argv;
#ifndef Q_WS_MAC
  return ((int*)(&Q_WS_MAC))[argc];
#else
  (void)argc;
  return 0;
#endif
}

