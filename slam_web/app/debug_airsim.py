# app/debug_airsim.py
import airsim
print("START")
client = airsim.MultirotorClient(ip="172.20.144.1")
client.confirmConnection()
print("CONNECTED")
