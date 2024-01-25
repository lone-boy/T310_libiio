#include <iostream>
#include "antsdrDevice.h"
#include "unistd.h"

#define DEVICE 0
void get_rx_data(sdr_transfer *trans){
    fprintf(stdout,"get samples=%d\n",trans->length);
}

int main() {
    antsdrDevice  device;
    if(device.open() < 0){
        return -1;
    }
    fprintf(stdout,"true\n");

//    device.set_rx_freq(0,2.4e9);
//    device.set_rx_freq(1,4.8e9);
    double start_tx = 100e6;
    double end_tx = 6e9;
    int step = (end_tx - start_tx)/ 10e6;
    device.set_tx_freq(DEVICE,start_tx);
//    device.set_tx_freq(1,4.8e9);

//    device.set_rx_samprate(0,3e6);
//    device.set_rx_samprate(1,3e6);
    device.set_tx_samprate(DEVICE,3e6);
//    device.set_tx_samprate(1,4e6);
//    device.set_rx_gain(0,60);
    device.set_tx_gain(DEVICE,0);
//    fprintf(stdout,"rx0 freq = %lf rx1 freq = %lf\n",device.get_rx_freq(0),device.get_rx_freq(1));
//    fprintf(stdout,"tx0 freq = %lf tx1 freq = %lf\n",device.get_tx_freq(0),device.get_tx_freq(1));
//    fprintf(stdout,"rx fs = %lf tx fs = %lf\n",device.get_rx_samprate(0),device.get_tx_samprate(0));
//    fprintf(stdout,"rx1 fs = %lf tx1 fs = %lf\n",device.get_rx_samprate(1),device.get_tx_samprate(1));

    device.start_tx(1<<1);
    /**
     * 0 1 2 3
     * 1 1 1 1
     * 1 0 1 0
     * 1 1 0 0
     * 0 0 1 1
     * 0 1 0 1
     * */
//    device.start_rx(get_rx_data,(1<<0 | 1<<1 | 0<<2 | 0<<3),NULL,1024000);


    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_time;
    diff_time = end_time - start_time;
    int i = 1;
    int span = 0;
    while(1){
        diff_time = end_time - start_time;
        end_time = std::chrono::high_resolution_clock::now();
        if(diff_time.count() > 120 or span > 2){
            break;
        }
        usleep(100000);
        if(i < step){
            fprintf(stdout,"center_frequency=%lf Mhz\n",(start_tx+i*10e6) / 1e6);
            device.set_tx_freq(DEVICE,start_tx+i*10e6);
            i++;
        }
        else{
            i = 0;
            span++;
        }

    }
//    device.stop_rx();
    device.stop_tx();
    return 0;
}
