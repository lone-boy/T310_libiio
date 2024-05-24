//
// Created by jcc on 23-7-21.
//
#include "antsdrDevice.h"
#include <iostream>
#include <functional>
#include "ad9361.h"
#include "math.h"

antsdrDevice::antsdrDevice()
    :is_Inited_(false),rx_user_(NULL),rx_buf_(NULL),rx_thread_(NULL)
    ,rx_handler_(NULL), rx_running_(false)
{
    rxone0_i_ = NULL;
    rxone0_q_ = NULL;
    rxtwo0_i_ = NULL;
    rxtwo0_q_ = NULL;
    rxone1_i_ = NULL;
    rxone1_q_ = NULL;
    rxtwo1_i_ = NULL;
    rxtwo1_q_ = NULL;

    txone0_i_ = NULL;
    txone0_q_ = NULL;
    txtwo0_i_ = NULL;
    txtwo0_q_ = NULL;
    txone1_i_ = NULL;
    txone1_q_ = NULL;
    txtwo1_i_ = NULL;
    txtwo1_q_ = NULL;
}

antsdrDevice::~antsdrDevice() {

}

std::vector<int16_t> generateCosineSignal(double frequency, double sampleRate, double durationInSeconds) {
    std::vector<int16_t> signal;
    double angularFrequency = 2 * M_PI * frequency;
    double maxAmplitude = 65536.0; // Maximum positive value for int16_t

    int numSamples = static_cast<int>(durationInSeconds * sampleRate);

    for (int i = 0; i < numSamples; i++) {
        double time = static_cast<double>(i) / sampleRate;
        double sample = cos(angularFrequency * time);
        int16_t scaledSample = static_cast<int16_t>(round(sample * maxAmplitude));
        signal.push_back(scaledSample);
    }

    return signal;
}


int antsdrDevice::open() {
    if(is_Inited_)
        return 0;

    antsdr_ctx_ = iio_create_context_from_uri("ip:192.168.1.10");
    if(antsdr_ctx_ == NULL){
        fprintf(stderr,"cannot find device.\n");
        return -1;
    }

    phy_dev_one_ = iio_context_find_device(antsdr_ctx_,"ad9361-phy");
    if(phy_dev_one_ == NULL){
        fprintf(stderr,"cannot find ad9361 device.\n");
        return -1;
    }
    phy_dev_two_ = iio_context_find_device(antsdr_ctx_,"ad9361-phy-B");
    if(phy_dev_two_ == NULL){
        fprintf(stderr,"cannot find ad9361-B device.\n");
        return -1;
    }
    /**
     * Init phy ad9361 one
     * */
    phy_rx_one_chn0_ = iio_device_find_channel(phy_dev_one_,"voltage0", false);
    if(phy_rx_one_chn0_ == NULL){
        fprintf(stderr,"cannot find ad9361 one rx ch0.\n");
        return -1;
    }
    phy_rx_one_chn1_ = iio_device_find_channel(phy_dev_one_,"voltage1", false);
    if(phy_rx_one_chn1_ == NULL){
        fprintf(stderr,"cannot find ad9361 one rx ch1.\n");
        return -1;
    }

    phy_tx_one_chn0_ = iio_device_find_channel(phy_dev_one_,"voltage0", true);
    if(phy_tx_one_chn0_ == NULL){
        fprintf(stderr,"cannot find ad9361 one tx ch0.\n");
        return -1;
    }
    phy_tx_one_chn1_ = iio_device_find_channel(phy_dev_one_,"voltage1", true);
    if(phy_tx_one_chn1_ == NULL){
        fprintf(stderr,"cannot find ad9361 one tx ch1.\n");
        return -1;
    }

    /**
     * Init phy ad9361B one
     * */
    phy_rx_two_chn0_ = iio_device_find_channel(phy_dev_two_,"voltage0", false);
    if(phy_rx_two_chn0_ == NULL){
        fprintf(stderr,"cannot find ad9361B one rx ch0.\n");
        return -1;
    }
    phy_rx_two_chn1_ = iio_device_find_channel(phy_dev_two_,"voltage1", false);
    if(phy_rx_two_chn1_ == NULL){
        fprintf(stderr,"cannot find ad9361B one rx ch1.\n");
        return -1;
    }

    phy_tx_two_chn0_ = iio_device_find_channel(phy_dev_two_,"voltage0", true);
    if(phy_tx_two_chn0_ == NULL){
        fprintf(stderr,"cannot find ad9361B one tx ch0.\n");
        return -1;
    }
    phy_tx_two_chn1_ = iio_device_find_channel(phy_dev_two_,"voltage1", true);
    if(phy_tx_two_chn1_ == NULL){
        fprintf(stderr,"cannot find ad9361B one tx ch1.\n");
        return -1;
    }

    /* ad9361 rx tx stream */
    antsdr_rx_one_ = iio_context_find_device(antsdr_ctx_,CAP_DEVICE1);
    if(antsdr_rx_one_ == NULL){
        fprintf(stderr,"cannot find ad9361 one rx iq.\n");
        return -1;
    }
    /* ad9361 rxb tx stream */
    antsdr_rx_two_ = iio_context_find_device(antsdr_ctx_,CAP_DEVICE2);
    if(antsdr_rx_two_ == NULL){
        fprintf(stderr,"cannot find ad9361B rx iq.\n");
        return -1;
    }

    antsdr_tx_one_ = iio_context_find_device(antsdr_ctx_,DDS_DEVICE1);
    if(antsdr_tx_one_ == NULL){
        fprintf(stderr,"cannot find ad9361A tx iq.\n");
        return -1;
    }

    antsdr_tx_two_ = iio_context_find_device(antsdr_ctx_,DDS_DEVICE2);
    if(antsdr_tx_two_ == NULL){
        fprintf(stderr,"cannot find ad9361B tx iq.\n");
        return -1;
    }

    iio_channel_attr_write(phy_rx_one_chn0_, "gain_control_mode", "manual");
    iio_channel_attr_write(phy_rx_two_chn0_, "gain_control_mode", "manual");
    fprintf(stdout,"Find two ad9361 device.\nWelcom to ANTSDR t310.\n");
    is_Inited_ = true;
    return 0;
}

bool antsdrDevice::set_rx_freq(int device_index, double freq) {
    if(not is_Inited_ or not phy_dev_one_ or not phy_dev_two_)
        return false;
    int r;
    switch (device_index) {
        case 0:
        {
            struct iio_channel *phy = iio_device_find_channel(phy_dev_one_,"altvoltage0",true);
            r = iio_channel_attr_write_longlong(phy,"frequency",(long long)freq);
            break;
        }
        case 1:{
            struct iio_channel *phy = iio_device_find_channel(phy_dev_two_,"altvoltage0",true);
            r = iio_channel_attr_write_longlong(phy,"frequency",(long long)freq);
            break;
        }
        default:
            fprintf(stderr,"Error input index. set index 0 or 1\n");
            return false;
    }
    return r == 0;
}

bool antsdrDevice::set_tx_freq(int device_index, double freq) {
    if(not is_Inited_ or not phy_dev_one_ or not phy_dev_two_)
        return false;
    int r;
    switch (device_index) {
        case 0:
        {
            struct iio_channel *phy = iio_device_find_channel(phy_dev_one_,"altvoltage1",true);
            r = iio_channel_attr_write_longlong(phy,"frequency",(long long)freq);
            break;
        }
        case 1:{
            struct iio_channel *phy = iio_device_find_channel(phy_dev_two_,"altvoltage1",true);
            r = iio_channel_attr_write_longlong(phy,"frequency",(long long)freq);
            break;
        }
        default:
            fprintf(stderr,"Error input index. set index 0 or 1\n");
            return false;
    }
    return r == 0;
}

double antsdrDevice::get_rx_freq(int device_index) {
    long long int frequency;
    if(not is_Inited_ or not phy_dev_one_ or not phy_dev_two_)
        return false;
    int r;
    if(device_index == 0){
        struct iio_channel *phy = iio_device_find_channel(phy_dev_one_,"altvoltage0",true);
        r = iio_channel_attr_read_longlong(phy,"frequency",&frequency);
        if(r == 0)
            return (double)frequency;
    }
    if(device_index == 1){
        struct iio_channel *phy = iio_device_find_channel(phy_dev_two_,"altvoltage0",true);
        r = iio_channel_attr_read_longlong(phy,"frequency",&frequency);
        if(r == 0)
            return (double)frequency;
    }
    return 0;
}

double antsdrDevice::get_tx_freq(int device_index) {
    long long int frequency;
    if(not is_Inited_ or not phy_dev_one_ or not phy_dev_two_)
        return false;
    int r;
    if(device_index == 0){
        struct iio_channel *phy = iio_device_find_channel(phy_dev_one_,"altvoltage1",true);
        r = iio_channel_attr_read_longlong(phy,"frequency",&frequency);
        if(r == 0)
            return (double)frequency;
    }
    if(device_index == 1){
        struct iio_channel *phy = iio_device_find_channel(phy_dev_two_,"altvoltage1",true);
        r = iio_channel_attr_read_longlong(phy,"frequency",&frequency);
        if(r == 0)
            return (double)frequency;
    }
    return 0;
}

bool antsdrDevice::set_rx_samprate(int device_index, double fs) {
    if(not is_Inited_)
        return false;
    int r;
    switch (device_index) {
        case 0:{
            r = iio_channel_attr_write_longlong(phy_rx_one_chn0_,"sampling_frequency",fs);
            iio_channel_attr_write_longlong(phy_rx_one_chn0_, "rf_bandwidth", fs);
            break;
        }
        case 1:{
            r = iio_channel_attr_write_longlong(phy_rx_two_chn0_,"sampling_frequency",fs);
            iio_channel_attr_write_longlong(phy_rx_two_chn0_, "rf_bandwidth", fs);
            break;
        }
        default:{
            fprintf(stderr,"Error input index. set index 0 or 1\n");
            return false;
        }
    }
    return r==0;
}

bool antsdrDevice::set_tx_samprate(int device_index, double fs) {
    if(not is_Inited_ or not phy_tx_one_chn0_)
        return false;
    int r;
    switch (device_index) {
        case 0:{
            r = iio_channel_attr_write_longlong(phy_tx_one_chn0_,"sampling_frequency",fs);
            iio_channel_attr_write_longlong(phy_tx_one_chn0_, "rf_bandwidth", fs);
            break;
        }
        case 1:{
            r = iio_channel_attr_write_longlong(phy_tx_two_chn0_,"sampling_frequency",fs);
            iio_channel_attr_write_longlong(phy_tx_two_chn0_, "rf_bandwidth", fs);

            break;
        }
        default:{
            fprintf(stderr,"Error input index. set index 0 or 1\n");
            return false;
        }
    }
    return r==0;
}

double antsdrDevice::get_rx_samprate(int device_index) {
    long long int fs;
    if(not is_Inited_ or not phy_dev_one_ or not phy_dev_two_)
        return false;
    int r;
    if(device_index == 0){
        r = iio_channel_attr_read_longlong(phy_rx_one_chn0_,"sampling_frequency",&fs);
        if(r == 0)
            return (double)fs;
    }
    if(device_index == 1){

        r = iio_channel_attr_read_longlong(phy_rx_two_chn0_,"sampling_frequency",&fs);
        if(r == 0)
            return (double)fs;
    }
    return 0;
}

double antsdrDevice::get_tx_samprate(int device_index) {
    long long int fs;
    if(not is_Inited_ or not phy_dev_one_ or not phy_dev_two_)
        return false;
    int r;
    if(device_index == 0){
        r = iio_channel_attr_read_longlong(phy_tx_one_chn0_,"sampling_frequency",&fs);
        if(r == 0)
            return (double)fs;
    }
    if(device_index == 1){
        r = iio_channel_attr_read_longlong(phy_tx_two_chn0_,"sampling_frequency",&fs);
        if(r == 0)
            return (double)fs;
    }
    return 0;
}

bool antsdrDevice::start_rx(RXdataCallback handler,int channels,void *user,int buff_size) {
    rx_handler_ = handler;
    rx_user_ = user;
    bool config_flag = false;
    buffer_size_ = buff_size;
    if(not is_Inited_)
        return false;
    printf("channels=%d\n",channels);

    /* iio_channel_enable */
    if(channels & 0x1){
        config_stream_device(&rxone0_i_,0, false,antsdr_rx_one_);
        config_stream_device(&rxone0_q_,1, false,antsdr_rx_one_);

    }
    if(channels  & 0x2){
        config_stream_device(&rxone1_i_,2, false,antsdr_rx_one_);
        config_stream_device(&rxone1_q_,3, false,antsdr_rx_one_);
    }
    if(channels & 0x4){
        config_stream_device(&rxtwo0_i_,4, false,antsdr_rx_one_);
        config_stream_device(&rxtwo0_q_,5, false,antsdr_rx_one_);
    }
    if(channels & 0x8){
        config_stream_device(&rxtwo1_i_,6, false,antsdr_rx_one_);
        config_stream_device(&rxtwo1_q_,7, false,antsdr_rx_one_);
    }
    if(! rx_running_ && !rx_thread_){
        rx_running_ = true;
        rx_thread_ = new std::thread(std::bind(&antsdrDevice::RXSyncThread,this,channels));
    }

    return rx_running_ & config_flag;
}

void antsdrDevice::RXSyncThread(int channels) {
    sdr_transfer trans;
    iio_device_set_kernel_buffers_count(antsdr_rx_one_,2);
    rx_buf_ = iio_device_create_buffer(antsdr_rx_one_,buffer_size_, false);
    while(rx_running_){
        int n_read = iio_buffer_refill(rx_buf_);
        if(n_read > 0){
            trans.data = (int16_t*) iio_buffer_start(rx_buf_);
            trans.user = rx_user_;
            trans.length = n_read / 4;
            trans.channels = channels;
            rx_handler_(&trans);
        }
        else{
            break;
        }

    }
    rx_running_ = false;
}


bool antsdrDevice::config_rx_stream_device0_rx0() {
    rxone0_i_ = iio_device_find_channel(antsdr_rx_one_,RXDATA0CHANNEL_I, false);
    if(rxone0_i_ == NULL){
        fprintf(stderr,"cannot find device0 0 i channel.\n");
        return false;
    }
    rxone0_q_ = iio_device_find_channel(antsdr_rx_one_,RXDATA0CHANNEL_Q, false);
    if(rxone0_q_ == NULL){
        fprintf(stderr,"cannot find device0 0 q channel.\n");
        return false;
    }
    iio_channel_enable(rxone0_i_);
    iio_channel_enable(rxone0_q_);
    fprintf(stderr,"find device0 0 iq channel.\n");
    return true;
}

bool antsdrDevice::config_rx_stream_device0_rx1() {
    rxone1_i_ = iio_device_find_channel(antsdr_rx_one_,RXDATA1CHANNEL_I, false);
    if(rxone1_i_ == NULL){
        fprintf(stderr,"cannot find device1 1 i channel.\n");
        return false;
    }
    rxone1_q_ = iio_device_find_channel(antsdr_rx_one_,RXDATA1CHANNEL_Q, false);
    if(rxone1_q_ == NULL){
        fprintf(stderr,"cannot find device1 1 q channel.\n");
        return false;
    }
    iio_channel_enable(rxone1_i_);
    iio_channel_enable(rxone1_q_);
    fprintf(stderr,"find device0 1 iq channel.\n");
    return true;
}

bool antsdrDevice::config_rx_stream_device1_rx0() {
    rxtwo0_i_ = iio_device_find_channel(antsdr_rx_one_,RXDATA2CHANNEL_I, false);
    if(rxtwo0_i_ == NULL){
        fprintf(stderr,"cannot find device1 0 i channel.\n");
        return false;
    }
    rxtwo0_q_ = iio_device_find_channel(antsdr_rx_one_,RXDATA2CHANNEL_Q, false);
    if(rxtwo0_q_ == NULL){
        fprintf(stderr,"cannot find device1 0 q channel.\n");
        return false;
    }
    iio_channel_enable(rxtwo0_i_);
    iio_channel_enable(rxtwo0_q_);
    fprintf(stderr,"find device1 0 iq channel.\n");
    return true;
}

bool antsdrDevice::config_rx_stream_device1_rx1() {
    rxtwo1_i_ = iio_device_find_channel(antsdr_rx_one_,RXDATA3CHANNEL_I, false);
    if(rxtwo1_i_ == NULL){
        fprintf(stderr,"cannot find device1 1 i channel.\n");
        return false;
    }
    rxtwo1_q_ = iio_device_find_channel(antsdr_rx_one_,RXDATA3CHANNEL_Q, false);
    if(rxtwo1_q_ == NULL){
        fprintf(stderr,"cannot find device1 1 q channel.\n");
        return false;
    }
    iio_channel_enable(rxtwo1_i_);
    iio_channel_enable(rxtwo1_q_);
    fprintf(stderr,"find device1 1 iq channel.\n");
    return true;
}

bool antsdrDevice::start_tx(int channels) {
    int channelsCnt = 0;
    if(channels & 0x1){
        config_stream_device(&txone0_i_,0, true,antsdr_tx_one_);
        config_stream_device(&txone0_q_,1, true,antsdr_tx_one_);
        channelsCnt++;
    }
    if(channels  & 0x2){
        config_stream_device(&txone1_i_,2, true,antsdr_tx_one_);
        config_stream_device(&txone1_q_,3, true,antsdr_tx_one_);
        channelsCnt++;
    }
    if(channels & 0x4){
        config_stream_device(&txtwo0_i_,4, true,antsdr_tx_one_);
        config_stream_device(&txtwo0_q_,5, true,antsdr_tx_one_);
        channelsCnt++;
    }
    if(channels & 0x8){
        config_stream_device(&txtwo1_i_,6, true,antsdr_tx_one_);
        config_stream_device(&txtwo1_q_,7, true,antsdr_tx_one_);
        channelsCnt++;
    }
    iio_device_set_kernel_buffers_count(antsdr_tx_one_,1);

    /* create sin wave */
    double frequency = 100e3;
    double samprate = 3e6;
    double duration = 1.0 / frequency * 30;
    std::vector<int16_t> wave_signal = generateCosineSignal(frequency,samprate,duration);


    tx_buf_ = iio_device_create_buffer(antsdr_tx_one_,wave_signal.size(), true);
    auto *tx_buffer = (int16_t*) iio_buffer_start(tx_buf_);
    for(int i = 0; i<wave_signal.size();i++){
        tx_buffer[2*i] =  wave_signal[i];
        tx_buffer[2*i+1] =  0;
    }
    ssize_t n_write = iio_buffer_push(tx_buf_);
    if(n_write < 0){
        fprintf(stderr,"tx send failed\n");
        return false;
    }else{
        fprintf(stdout,"tx send %d samples.\n",n_write / 4);
    }
    return true;
}

bool antsdrDevice::config_stream_device(iio_channel **channel, int chid, bool tx,struct iio_device *dev) {
    *channel = iio_device_find_channel(dev, get_chan_name("voltage",chid),tx);
    if (!*channel)
        *channel = iio_device_find_channel(dev, get_chan_name("altvoltage", chid), tx);
    printf("find channel %d\n",chid);

    if(*channel != NULL)
        iio_channel_enable(*channel);
    return *channel != NULL;
}

const char* antsdrDevice::get_chan_name(const char *type, int id) {
    snprintf(tmpstr_,sizeof(tmpstr_),"%s%d",type,id);
    return tmpstr_;
}

bool antsdrDevice::set_multichip_phase_sync(long long lo){
    if(antsdr_ctx_ == NULL)
        return false;


    return ad9361_fmcomms5_phase_sync(antsdr_ctx_,lo);
}

bool antsdrDevice::set_rx_gain(int device_index, int channel,double gain) {
    int r;
    if(device_index == 0){
        if(channel == 0)
            r = iio_channel_attr_write_double(phy_rx_one_chn0_, "hardwaregain", gain);
        else
            r = iio_channel_attr_write_double(phy_rx_one_chn1_, "hardwaregain", gain);

    }
    else{
        if(channel == 0)
            r = iio_channel_attr_write_double(phy_rx_two_chn0_, "hardwaregain", gain);
        else
            r = iio_channel_attr_write_double(phy_rx_two_chn1_, "hardwaregain", gain);
    }
    return r == 0;
}

void antsdrDevice::disable_chan(iio_channel **chan) {
    if(*chan != NULL)
        iio_channel_disable(*chan);
    *chan = NULL;
}


void antsdrDevice::stop_rx() {
    rx_running_ = false;
    if(rx_thread_ && rx_thread_->joinable()) {
        rx_thread_->join();
        delete (rx_thread_);
        rx_thread_ = NULL;
    }

    disable_chan(&rxone0_i_);
    disable_chan(&rxone0_q_);
    disable_chan(&rxone1_i_);
    disable_chan(&rxone1_i_);
    disable_chan(&rxtwo0_i_);
    disable_chan(&rxtwo0_q_);
    disable_chan(&rxtwo1_i_);
    disable_chan(&rxtwo1_q_);
    if(rx_buf_){
        iio_buffer_destroy(rx_buf_);
        rx_buf_ = NULL;
    }
}

void antsdrDevice::stop_tx() {
    disable_chan(&txone0_i_);
    disable_chan(&txone0_q_);
    disable_chan(&txone1_i_);
    disable_chan(&txone1_i_);
    disable_chan(&txtwo0_i_);
    disable_chan(&txtwo0_q_);
    disable_chan(&txtwo1_i_);
    disable_chan(&txtwo1_q_);
    if(tx_buf_){
        iio_buffer_destroy(tx_buf_);
        tx_buf_ = NULL;
    }

}
