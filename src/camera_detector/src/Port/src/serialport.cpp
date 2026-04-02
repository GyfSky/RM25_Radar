#include "../include/serialport.h"


/**
 *@brief   初始化数据
 *@param  fd       类型  int  打开的串口文件句柄
 *@param  speed    类型  int  波特率

 *@param  databits 类型  int  数据位   取值 为 7 或者8

 *@param  stopbits 类型  int  停止位   取值为 1 或者2
 *@param  parity   类型  int  效验类型 取值为N,E,O,S
 *@param  portchar 类型  char* 串口路径
 */
bool SerialPort::initSerialPort(const char *port_path,std::string config_path){

    YAML::Node config = YAML::LoadFile(config_path);
    std::cout << "give sudo to port" << std::endl;
	std::string command = "echo \\\' | sudo -S chmod 777 ";
	command += port_path;
	std::cout<<"potr command: "<<command<< std::endl;
    system(command.c_str());

    std::cout << "finish sudo to port" << std::endl;
    this->fd = open(port_path, O_RDWR|O_NOCTTY| O_NDELAY);
    speed = config["port"]["speed"].as<int>();
    dataBits = config["port"]["dataBits"].as<int>();
    stopBits = config["port"]["stopBits"].as<int>();
    parity = config["port"]["parity"].as<char>();

    set_Brate();
	if (set_Bit() == FALSE){
        printf("Set Parity Error\n");
		exit(0);
    }
    printf("Open successed\n");
    set_Bit();

    return true;
}

/**
 *@brief   设置波特率
 */
void SerialPort::set_Brate(){

    int speed_arr[] = {B921600, B460800, B230400, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};
    int name_arr[] = {921600, 460800, 230400,115200, 38400, 19200, 9600, 4800, 2400, 1200,  300};
    int   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);

	for (i = 0;  i < sizeof(speed_arr) / sizeof(int);  i++){
		if (speed == name_arr[i]){
			tcflush(fd, TCIOFLUSH);//清空缓冲区的内容
			cfsetispeed(&Opt, speed_arr[i]);//设置接受和发送的波特率
			cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt); //使设置立即生效

			if (status != 0){
                perror("tcsetattr fd1");
                return;
            }
			tcflush(fd, TCIOFLUSH);
        }
    }
}

/**
 *@brief   设置串口数据位，停止位和效验位
 */
int SerialPort::set_Bit(){
    struct termios termios_p;

	if (tcgetattr(fd, &termios_p)  !=  0){
        perror("SetupSerial 1");
		return (FALSE);
    }

    termios_p.c_cflag |= (CLOCAL | CREAD);  //接受数据
	termios_p.c_cflag &= ~CSIZE;//设置数据位数

    switch (dataBits)
    {
	case 7:
		termios_p.c_cflag |= CS7;
		break;

	case 8:
		termios_p.c_cflag |= CS8;
		break;

	default:
		fprintf(stderr, "Unsupported data size\n");
		return (FALSE);
    }

	//设置奇偶校验位double
    switch (parity)
    {
	case 'n':
	case 'N':
		termios_p.c_cflag &= ~PARENB;   /* Clear parity enable */
		termios_p.c_iflag &= ~INPCK;     /* Enable parity checking */
		break;

	case 'o':
	case 'O':
		termios_p.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
		termios_p.c_iflag |= INPCK;             /* Disnable parity checking */
		break;

	case 'e':
	case 'E':
		termios_p.c_cflag |= PARENB;     /* Enable parity */
		termios_p.c_cflag &= ~PARODD;   /* 转换为偶效验*/
		termios_p.c_iflag |= INPCK;       /* Disnable parity checking */
		break;

	case 'S':
	case 's':  /*as no parity*/
		termios_p.c_cflag &= ~PARENB;
		termios_p.c_cflag &= ~CSTOPB;
		break;

	default:
		fprintf(stderr, "Unsupported parity\n");
		return (FALSE);

    }

    /* 设置停止位*/
    switch (stopBits)
    {
	case 1:
		termios_p.c_cflag &= ~CSTOPB;
		break;

	case 2:
		termios_p.c_cflag |= CSTOPB;
		break;

	default:
		fprintf(stderr, "Unsupported stop bits\n");
		return (FALSE);

    }

    /* Set input parity option */
    if (parity != 'n')
        termios_p.c_iflag |= INPCK;

	tcflush(fd, TCIFLUSH); //清除输入缓存区 https://blog.csdn.net/qq39221093/article/details/35561865
    termios_p.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    termios_p.c_cc[VMIN] = 0;  //最小接收字符
    termios_p.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input原始输入*/
	termios_p.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    termios_p.c_iflag &= ~(ICRNL | IGNCR);
    termios_p.c_oflag &= ~OPOST;   /*Output禁用输出处理*/

	if (tcsetattr(fd, TCSANOW, &termios_p) != 0){ /* Update the options and do it NOW */
        perror("SetupSerial 3");
        return (FALSE);
    }

    return (TRUE);
}