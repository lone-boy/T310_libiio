#include <iostream>
#include "antsdrDevice.h"
#include "unistd.h"
void get_rx_data(sdr_transfer *trans){
    fprintf(stdout,"get samples=%d\n",trans->length);
}

int main() {
    antsdrDevice  device;
    if(device.open() < 0){
        fprintf(stdout,"false\n");
        return -1;
    }
    fprintf(stdout,"true\n");

    device.set_rx_freq(0,2.4e9);
    device.set_rx_freq(1,4.8e9);
    device.set_tx_freq(0,2.4e9);
    device.set_tx_freq(1,4.8e9);

    device.set_rx_samprate(0,3e6);
    device.set_rx_samprate(1,3e6);
    device.set_tx_samprate(0,3e6);
    device.set_tx_samprate(1,4e6);
    fprintf(stdout,"rx0 freq = %lf rx1 freq = %lf\n",device.get_rx_freq(0),device.get_rx_freq(1));
    fprintf(stdout,"tx0 freq = %lf tx1 freq = %lf\n",device.get_tx_freq(0),device.get_tx_freq(1));
    fprintf(stdout,"rx fs = %lf tx fs = %lf\n",device.get_rx_samprate(0),device.get_tx_samprate(0));
    fprintf(stdout,"rx1 fs = %lf tx1 fs = %lf\n",device.get_rx_samprate(1),device.get_tx_samprate(1));

    /**
     * 0 1 2 3
     * 1 1 1 1
     * 1 0 1 0
     * 1 1 0 0
     * 0 0 1 1
     * 0 1 0 1
     * */
    device.start_rx(get_rx_data,(1<<0 | 1<<1 | 1<<2 | 1<<3),NULL,1024000);


    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_time;
    diff_time = end_time - start_time;
    while(1){
        diff_time = end_time - start_time;
        end_time = std::chrono::high_resolution_clock::now();
        if(diff_time.count() > 5){
            break;
        }
        sleep(1);
    }
    return 0;
}
