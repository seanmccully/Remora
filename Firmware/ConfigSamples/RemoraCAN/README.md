Copy the following to create the two files for setting up Linux CAN Network Interface. Ensure systemd-networkd is enabled at boot-up. 
Otherwise configure the CAN network device as you prefer. Currently the BitRate and SamplePoint is compiled in. 


    $ cat <<EOF >> /etc/systemd/network/00-can0.network
    [Match]
    Name=can*

    [CAN]
    BitRate=500K
    SamplePoint=75%
    EOF

    $ cat <<EOF >> /etc/systemd/network/00-can0.link
    [Match]
    Type=can

    [Link]
    TransmitQueues=2048
    ReceiveQueues=2048
    TransmitQueueLength=1024
    EOF

    $ systemd enable systemd-networkd
    $ systemd [re]start systemd-networkd
