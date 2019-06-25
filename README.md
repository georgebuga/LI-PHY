# LI-PHY

TX/RX code for LiFi data transmition
TX code will handle data, scrambler with 0xAA sequence, and compute a packet as below:
10 bytes as sincronization pilots + 1 byte as data length + data

RX wait untill all 10 sincronization bytes will match , read data lenght and store data.
