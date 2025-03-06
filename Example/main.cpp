#include <iostream>
#include "Interface/vehicle.h"

int main()
{
	// std::string ipAddress = "192.168.1.40";
	// int port = 9003;
	std::string ipAddress = "192.168.44.128";
	int port = 9003;
	std::cout<<ipAddress<<" : "<<port<<std::endl;
	Vehicle *device = new Vehicle(ipAddress, port);
	device->Run();

	getchar();
	return 0;
}
