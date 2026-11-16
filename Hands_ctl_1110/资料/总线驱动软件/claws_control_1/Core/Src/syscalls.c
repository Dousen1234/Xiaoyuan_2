/* No UART/semihosting backend is connected on this board. */
#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
int _close(int fd) { (void)fd; errno = EBADF; return -1; }
int _fstat(int fd, struct stat *st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
int _lseek(int fd, int off, int whence) { (void)fd; (void)off; (void)whence; errno = ESPIPE; return -1; }
int _read(int fd, char *p, int n) { (void)fd; (void)p; (void)n; errno = ENOSYS; return -1; }
int _write(int fd, char *p, int n) { (void)fd; (void)p; (void)n; errno = ENOSYS; return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
void *_sbrk(ptrdiff_t n) { (void)n; errno = ENOMEM; return (void *)-1; }
void _exit(int code) { (void)code; for (;;) {} }
