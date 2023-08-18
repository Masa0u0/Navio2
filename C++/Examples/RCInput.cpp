#include <memory>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include <Common/Util.h>
#include <Navio+/RCInput_Navio.h>
#include <Navio2/RCInput_Navio2.h>

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

int main()
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
    cout << endl;
    sleep(1);
  }

  return 0;
}
