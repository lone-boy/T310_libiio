#include <iostream>
#include "antsdrDevice.h"
#include "unistd.h"
#include "vector"

std::vector<double> x_data,recv1_i,recv1_q;
std::vector<double> recv2_i,recv2_q;
std::vector<double> recv3_i,recv3_q;
std::vector<double> recv4_i,recv4_q;

pthread_mutex_t lock;
bool draw = false;

void get_rx_data(sdr_transfer *trans){
    pthread_mutex_lock(&lock);

//    fprintf(stdout,"rece channel %d get samples=%d\n",trans->channels,trans->length);
    if(trans->channels == 15){
        /* 4 channels open */
        recv1_i.clear();
        recv1_q.clear();
        recv2_i.clear();
        recv2_q.clear();
        recv3_i.clear();
        recv3_q.clear();
        recv4_i.clear();
        recv4_q.clear();

        for(int i=0;i<trans->length / 4;i++){
            recv1_i.push_back(trans->data[(2*4)*i]);
            recv1_q.push_back(trans->data[(2*4)*i+1]);
            recv2_i.push_back(trans->data[(2*4)*i+2]);
            recv2_q.push_back(trans->data[(2*4)*i+3]);
            recv3_i.push_back(trans->data[(2*4)*i+4]);
            recv3_q.push_back(trans->data[(2*4)*i+5]);
            recv4_i.push_back(trans->data[(2*4)*i+6]);
            recv4_q.push_back(trans->data[(2*4)*i+7]);
        }
        draw = true;
    }
    pthread_mutex_unlock(&lock);

}

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main() {
    antsdrDevice  device;
    if(device.open() < 0){
        return -1;
    }

    device.set_rx_gain(0,0,0);
    device.set_rx_gain(0,1,0);

    device.set_rx_gain(1,0,0);
    device.set_rx_gain(1,1,0);

    double freq = 5e9;



    device.set_rx_freq(0,freq);
    device.set_rx_freq(1,freq);
//    device.set_tx_freq(0,2.4e9);
//    device.set_tx_freq(1,4.8e9);

    device.set_rx_samprate(0,4e6);
    device.set_rx_samprate(1,4e6);

    if(device.set_multichip_phase_sync(freq) == 0){
        fprintf(stdout,"multichip sync true\n");
    }
    else{
        fprintf(stdout,"multichip sync false\n");
    }
//    device.set_tx_samprate(0,4e6);
//    device.set_tx_samprate(1,4e6);
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
     /*  */
     int samples = 4096;
    device.start_rx(get_rx_data,(1<<0 | 1<<1 | 1<<2 | 1<<3),NULL,samples);


    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_time;
    diff_time = end_time - start_time;

    for(int i=0;i<samples;i++){
        x_data.push_back(i);
    }
    plt::figure_size(900,300);
    while(1){
        pthread_mutex_lock(&lock);
//        diff_time = end_time - start_time;
//        end_time = std::chrono::high_resolution_clock::now();
//        if(diff_time.count() > 5){
//            break;
//        }
        if(draw){
            plt::figure(1);
            plt::clf();
            plt::plot(x_data,recv1_i,"b");
            plt::plot(x_data,recv1_q,"g");

            plt::plot(x_data,recv2_i,"r");
            plt::plot(x_data,recv2_q,"y");

            plt::draw();

            plt::figure(2);
            plt::clf();
            plt::plot(x_data,recv3_i,"b");
            plt::plot(x_data,recv3_q,"g");
            plt::plot(x_data,recv4_i,"r");
            plt::plot(x_data,recv4_q,"y");


            plt::draw();
            plt::pause(0.1);

            draw = false;
        }

        pthread_mutex_unlock(&lock);
//        sleep(1);
    }
    return 0;
}
