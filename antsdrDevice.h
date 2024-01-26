//
// Created by jcc on 23-7-21.
//

#ifndef T310_LIBIIO_ANTSDRDEVICE_H
#define T310_LIBIIO_ANTSDRDEVICE_H

#include "iio.h"
#include "thread"
#include <cstring>
#include "math.h"


#define RXDATA0CHANNEL_I "voltage0"
#define RXDATA0CHANNEL_Q "voltage1"
#define RXDATA1CHANNEL_I "voltage2"
#define RXDATA1CHANNEL_Q "voltage3"

#define RXDATA2CHANNEL_I "voltage4"
#define RXDATA2CHANNEL_Q "voltage5"
#define RXDATA3CHANNEL_I "voltage6"
#define RXDATA3CHANNEL_Q "voltage7"

#define CAP_DEVICE1 "cf-ad9361-A"
#define CAP_DEVICE2 "cf-ad9361-B"
#define DDS_DEVICE1 "cf-ad9361-dds-core-lpc"
#define DDS_DEVICE2 "cf-ad9361-dds-core-B"

typedef struct
{
    void *user;
    int16_t *data;
    int channels;
    int length;
} sdr_transfer;

typedef void(*RXdataCallback)(sdr_transfer *transfer);

class antsdrDevice{
public:
    antsdrDevice();
    ~antsdrDevice();

    int open();
    bool set_rx_freq(int device_index,double freq);
    bool set_tx_freq(int device_index,double freq);
    double get_rx_freq(int device_index);
    double get_tx_freq(int device_index);
    bool set_rx_samprate(int device_index,double fs);
    bool set_tx_samprate(int device_index,double fs);
    double get_rx_samprate(int device_index);
    double get_tx_samprate(int device_index);
    bool set_rx_gain(int device_index,double gain);
    bool set_tx_gain(int device_index,double gain);

    void stop_rx();
    void stop_tx();

    bool set_rx_gain(int device_index,int channel,double gain);


    bool set_multichip_phase_sync(long long lo);


    /**
     * ad9361B      ad9361
     * rx2 rx1      rx2 rx1
     * 1    1       1   1   enable
     * 0    0       0   0   disable
     * */
    bool start_rx(RXdataCallback handler,int channels,void *user,int buff_size);
    void RXSyncThread(int channels);

    bool start_tx(int channels);
private:
    struct iio_context *antsdr_ctx_;
    struct iio_device  *antsdr_rx_one_;
    struct iio_device  *antsdr_rx_two_;
    struct iio_device  *antsdr_tx_one_;
    struct iio_device  *antsdr_tx_two_;
    struct iio_device  *phy_dev_one_;
    struct iio_device  *phy_dev_two_;
    struct iio_buffer  *rx_buf_;
    struct iio_buffer  *tx_buf_;
    struct iio_channel *phy_rx_one_chn0_,*phy_rx_one_chn1_;
    struct iio_channel *phy_tx_one_chn0_,*phy_tx_one_chn1_;
    struct iio_channel *phy_rx_two_chn0_,*phy_rx_two_chn1_;
    struct iio_channel *phy_tx_two_chn0_,*phy_tx_two_chn1_;

    struct iio_channel *rxone0_i_,*rxone0_q_;
    struct iio_channel *rxtwo0_i_,*rxtwo0_q_;
    struct iio_channel *rxone1_i_,*rxone1_q_;
    struct iio_channel *rxtwo1_i_,*rxtwo1_q_;

    struct iio_channel *txone0_i_,*txone0_q_;
    struct iio_channel *txone1_i_,*txone1_q_;
    struct iio_channel *txtwo0_i_,*txtwo0_q_;
    struct iio_channel *txtwo1_i_,*txtwo1_q_;


    bool is_Inited_;
    bool rx_running_;
    void *rx_user_;
    int64_t buffer_size_;
    RXdataCallback rx_handler_;
    std::thread *rx_thread_;
    char tmpstr_[64];


private:

    bool config_stream_device(iio_channel **channel,int chid,bool tx,struct iio_device *dev);
    void disable_chan(iio_channel ** chan);
    const char* get_chan_name(const char *type,int id);
};

#endif //T310_LIBIIO_ANTSDRDEVICE_H
