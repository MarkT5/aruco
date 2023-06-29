from KUKA import YouBot
robot = YouBot("192.168.88.22")
print(robot.ssh.send_wait("rostopic list", "/"))
