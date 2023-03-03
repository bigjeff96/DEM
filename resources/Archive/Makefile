
springbrake: main.o Plan.o Drum.o Sphere.o Cell.o Grid.o Contact.o Functions.o
	g++ -std=c++11 -W -Wall -O3 main.o Plan.o Drum.o Sphere.o Cell.o Grid.o Contact.o Functions.o -o springbrake

main.o: main.cpp Functions.h Sphere.h Plan.h Grid.h Contact.h Cell.h Drum.h
	g++ -std=c++11 -W -Wall -O3 -c main.cpp

Plan.o: Plan.cpp Plan.h
	g++ -std=c++11 -W -Wall -O3 -c Plan.cpp

Drum.o: Drum.cpp Drum.h
	g++ -std=c++11 -W -Wall -O3 -c Drum.cpp
	
Sphere.o: Sphere.cpp Sphere.h
	g++ -std=c++11 -W -Wall -O3 -c Sphere.cpp
	
Cell.o: Cell.cpp Cell.h Sphere.h Contact.h
	g++ -std=c++11 -W -Wall -O3 -c Cell.cpp

Grid.o: Grid.cpp Grid.h Cell.h Sphere.h
	g++ -std=c++11 -W -Wall -O3 -c Grid.cpp

Contact.o: Contact.cpp Contact.h Cell.h Sphere.h Plan.h Drum.h
	g++ -std=c++11 -W -Wall -O3 -c Contact.cpp

Functions.o: Functions.cpp Functions.h Sphere.h Plan.h Grid.h Contact.h Cell.h Drum.h
	g++ -std=c++11 -W -Wall -O3 -c Functions.cpp

clean:
	rm *.o springbrake
	
remove:
	rm grain*.txt
