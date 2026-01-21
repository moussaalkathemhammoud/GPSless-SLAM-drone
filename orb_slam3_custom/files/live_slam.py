import airsim

client = airsim.MultirotorClient(ip="172.20.144.1")
client.confirmConnection()

imgs = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Scene),
    airsim.ImageRequest("1", airsim.ImageType.Scene)
])

print("Left:", imgs[0].width, imgs[0].height)
print("Right:", imgs[1].width, imgs[1].height)
