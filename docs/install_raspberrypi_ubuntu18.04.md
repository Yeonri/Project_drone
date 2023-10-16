# Install_raspberry_ubuntu 20.04 LTS

하드웨어: Raspberry pi 4B 8Gb

- 운영체제: Ubuntu 18.04
  - ubuntu 18.04 운영체제 파일 (ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img)
  - https://cdimage.ubuntu.com/releases/18.04/release/
  - ROS-Melodic


- 운영체제: 운영체제: Ubuntu 20.04
  - ROS-Noetic



- 우분투 이미지 파일 설치 프로그램
  - https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#2-prepare-the-sd-card

모니터 없는 환경 - 와이파이로 ssh 연결
1. 설치 후 폴더 내에 ssh 파일 생성
2. network-config 파일 수정

```
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      "myhomewifi":
        password: "mypassword"
```
3. PUTTY를 이용해 라즈베리파이가 연결된 ip로 연결
4. 필요 라이브러리 설치
```
# GUI 필요시 설치
sudo apt-get install ubuntu-desktop

# window 원격 데스크톱 연결 설정
sudo apt-get install xrdp
sudo systemctl status xrdp
sudo adduser xrdp ssl-cert
sudo systemctl restart xrdp
sudo systemctl enable  xrdp
sudo ufw allow 3389
sudo apt search xrdp
sudo apt-get install xorgxrdp
```
5. 라즈베리파이 재부팅 후 window 원격 데스크톱에서 연결

