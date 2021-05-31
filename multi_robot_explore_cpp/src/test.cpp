#include <vector>
#include <array>
#include <iostream>

int main(int argc, char** argv){

    //std::vector<int> input = {1,2,3,4,5};
    //const int array_size = input.size();
    //std::array<int, array_size> int_array;
    int8_t i8t = 70;
    if((int)i8t < 71){
        std::cout<<"i8t < 71"<<std::endl;
        std::cout<<i8t<<std::endl;
        std::cout<<(int)i8t<<std::endl;
    }else{
        std::cout<<"no"<<std::endl;
    }
    return 0;
}