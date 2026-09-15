#ifndef iqfile_h
#define iqfile_h

int initIqfile(char *optarg);
int initIqfileFormat(const char *fmt);
int runIqfileSample(void);
int runIqfileCancel(void);
int runIqfileClose(void);

#endif /* iqfile_h */
