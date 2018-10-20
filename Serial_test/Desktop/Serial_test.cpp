#include <stdio.h>
#include <string.h>

#define BUFSIZE 32

char serialPortFilename[] = "/dev/ttyACM1";

int main(int argc, char* argv[])
{
	FILE *serPort;
	if (argc) {
		serPort = fopen(argv[1], "w");
	} else {
		serPort = fopen(serialPortFilename, "w");
	}


	if (serPort == NULL)
	{
		printf("ERROR when opening serial connection");
		return 0;
	}

	char writeBuffer[] = "m: 1 s: 28";

	fwrite(writeBuffer, sizeof(char), sizeof(writeBuffer), serPort);

	fclose(serPort);

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

		fread(readBuffer, sizeof(char),32,serPort);

		if(sizeof(readBuffer) != 0)
		{
			printf(readBuffer);
		}
	}

	return 0;

}
