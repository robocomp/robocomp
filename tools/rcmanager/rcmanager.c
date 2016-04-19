#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#define BASE "/opt/robocomp/bin/rcmanager.py"

int main(int argc, char **argv) {
	int i;
	int length;
	char *str, *ptr_str;

	length = strlen(BASE) + 1;
	for (i = 1; i < argc; i++) {
		length += strlen(argv[i] + 1);
	}

	str = (char *)malloc(length*sizeof(char));
	ptr_str = str;

	sprintf(ptr_str, "%s ", BASE);
	ptr_str += strlen(BASE) + 1;

	for (i = 1; i < argc; i++) {
		sprintf(ptr_str, "%s ", argv[i]);
		ptr_str += strlen(argv[i]) + 1;
	}

	i = system(str);
	free(str);
	return i;
}
