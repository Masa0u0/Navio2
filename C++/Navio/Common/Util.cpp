#include <cstdio>
#include <stdarg.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>

#include "Util.h"

#define SCRIPT_PATH "../../../check_apm.sh"

int write_file(const char* path, const char* fmt, ...)
{
  errno = 0;

  int fd = ::open(path, O_WRONLY | O_CLOEXEC);
  if (fd == -1)
  {
    return -errno;
  }

  va_list args;
  va_start(args, fmt);

  int ret = ::vdprintf(fd, fmt, args);
  int errno_bkp = errno;
  ::close(fd);

  va_end(args);

  if (ret < 1)
  {
    return -errno_bkp;
  }

  return ret;
}

int read_file(const char* path, const char* fmt, ...)
{
  errno = 0;

  FILE* file = ::fopen(path, "re");
  if (!file)
    return -errno;

  va_list args;
  va_start(args, fmt);

  int ret = ::vfscanf(file, fmt, args);
  int errno_bkp = errno;
  ::fclose(file);

  va_end(args);

  if (ret < 1)
    return -errno_bkp;

  return ret;
}

bool check_apm()
{
  int ret = system("ps -AT | grep -c ap-timer > /dev/null");

  if (WEXITSTATUS(ret) <= 0)
  {
    fprintf(stderr, "APM is running. Can't launch the example\n");
    return true;
  }

  return false;
}

int get_navio_version()
{
  int version;
  read_file("/sys/firmware/devicetree/base/hat/product_id", "%x", &version);
  return version;
}

float decodeBinary32(unsigned int bin)
{
  int sign = bin >> 31 ? -1 : 1;
  int exponent = ((bin & 0x7f800000) >> 23) - 0x7F;
  unsigned int fraction = bin & 0x007fffff;

  unsigned int pow2 = 1 << 23;
  float fraction_decoded = 0.;
  for (unsigned int i = 0; i < 23; ++i)
  {
    fraction_decoded += static_cast<float>((fraction >> i) & 1) / pow2;
    pow2 = pow2 >> 1;
  }

  return sign * (1 + fraction_decoded) * std::pow(2, exponent);
}
