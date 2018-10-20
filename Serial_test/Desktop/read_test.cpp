#include <stdio.h>
#include <string.h>

#define BUFSIZE 128

char serialPortFilename[] = "/dev/ttyACM1";

int main(int argc, char* argv[])
{
	FILE *serPort;
	if (argc) {
		serPort = fopen(argv[1], "r");
	} else {
		serPort = fopen(serialPortFilename, "r");
	}
  char readBuffer[BUFSIZE];
  int numBytesRead;
	printf(serialPortFilename);
	printf(":\n");

	int counter = 0;
	while(counter++ < 10000)
	{
		memset(readBuffer, 0, BUFSIZE);

		fread(readBuffer, sizeof(char),BUFSIZE,serPort);

		if(sizeof(readBuffer) != 0)
		{
			printf(readBuffer);
		}
	}

	return 0;

}
