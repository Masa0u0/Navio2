#include <unistd.h>
#include <cstdio>
#include <iostream>

#include <Navio2/RCInput_Navio2.h>
#include <Navio+/RCInput_Navio.h>
#include <Common/Util.h>
#include <memory>

#define READ_FAILED -1

using namespace std;

unique_ptr<RCInput> get_rcin()
{
  if (get_navio_version() == NAVIO2)
  {
    auto ptr = unique_ptr<RCInput>{ new RCInput_Navio2() };
    return ptr;
  }
  else
  {
    auto ptr = unique_ptr<RCInput>{ new RCInput_Navio() };
    return ptr;
  }
}

int main(int argc, char* argv[])
{
  if (check_apm())
  {
    return 1;
  }

  auto rcin = get_rcin();

  rcin->initialize();

  while (true)
  {
    for (int ch = 0; ch < 14; ++ch)
    {
      int period = rcin->read(ch);
      if (period == READ_FAILED)
        return EXIT_FAILURE;
      cout << "Channel: " << ch << ", Period: " << period << endl;
    }

    sleep(0.1);
  }

  return 0;
}
