# Real arm

Connect with the real HDT Angler arm.

1. Configure interface and check connection to arm:

The IP of hdt arm is `192.168.0.100`

```bash
sudo ifconfig eth0:1 192.168.0.X up ping 192.168.0.102
```

## current problems

1. When the hdt arm is first started, use left pincer control signal (close) to open it. In some cases, if you use the right pincer control signal (open), it will stuck and the effort of pincer  will increase to 50.

2. In some cases, after you control the hdt arm using XBox joy, the controller can not receive new goal from moveit control program. You need to restart the arm.