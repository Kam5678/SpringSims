Documentation
-------------

https://lakin.ca/givr/

Compilation
-----------

## How to Install Dependencies (Ubuntu)

    sudo apt install cmake build-essential

## How to Build

    cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release
    cmake --build build

## How to Run

    build/simple
    
    If opened in Visual studio 2019, the cMake files will run automatically,
    probably will need openGL installed for the whole program to run though!
    
## Controls

    Press P for panel to show up
        - The Panel controls which simulations to play, how to reset simulations as well as switching between different chains
            - Currently there are 4 different Models:
                - Pendulum
                - Chain
                - Jellycube
                - Cloth
                
# Spring-Simulations
