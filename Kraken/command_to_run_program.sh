g++ main.cc -lwiringPi IO/COM/SPI/SPI.cc IO/COM/CAN/CAN.cc IO/COM/SPI/Specific/SPI_CAN.cc ctrl_main_gr3.cc regulation/speed_controller_gr3.cc CtrlStruct_gr3.cc -o main && ./main
