spots={         
    'v0':[35,0,5],                                
    'v1':[-45,43,5],
    'v2':[-45,0,5],
    'v3':[-45,-43,5],
    'v4':[115,43,5],
    'v5':[115,0,5],
    'v6':[115,-43,5],
}                        

# UAV0=[[-45.5,-0.5,4.5],[-45,-43,4.5],[115,-43,4.5]]
# UAV1=[[3,-3,2.5],[3,-3,2.5]]
# UAV2=[[-43.5,0,4.5],[115,0,4.5],[-45,0,4.5]]
# UAV1=[[3,0,2.5],[3,0,2.5]]
# UAV4=[[-44.5,0.5,4.5],[-45,43,4.5],[115,43,4.5]]
# UAV1=[[3,3,2.5],[3,3,2.5]]

UAV0=[spots['v2']+[0.5,-0.5,0],spots['v3']+[0.5,1,0],spots['v6']+[-2,1,0],spots['v5']+[-2,-0.5,0],spots['v0']+[0,-0.5,0]]
UAV1=[spots['v0']+[0,-1.5,0],spots['v5']+[-6,-1.5,0],spots['v6']+[-6,3,0],spots['v3']+[1.5,3,0],spots['v2']+[1.5,-1.5,0]]
UAV2=[spots['v2']+[-1.5,0,0],spots['v1']+[-1.5,3,0],spots['v4']+[6,3,0],spots['v6']+[6,-3,0],spots['v3']+[-1.5,-3,0]]
UAV3=[spots['v5']+[2,0,0],spots['v4']+[2,1,0],spots['v1']+[-0.5,1,0],spots['v3']+[-0.5,-1,0],spots['v6']+[2,-1,0]]
UAV4=[spots['v2']+[0.5,0.5,0],spots['v1']+[0.5,-1,0],spots['v4']+[-2,-1,0],spots['v5']+[-2,0.5,0],spots['v0']+[0,0.5,0]]
UAV5=[spots['v0']+[0,1.5,0],spots['v5']+[-6,1.5,0],spots['v4']+[-6,-3,0],spots['v1']+[1.5,-3,0],spots['v2']+[1.5,1.5,0]]

output=[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
for i in range(5):
    for j in range(3):
        output[i][j]=UAV0[i][j]+UAV0[i][j+3]
print("UAV0 =",output)


output=[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
for i in range(5):
    for j in range(3):
        output[i][j]=UAV0[i][j]+UAV0[i][j+3]
print("UAV0 =",output)

for i in range(5):
    for j in range(3):
        output[i][j]=UAV1[i][j]+UAV1[i][j+3]
print("UAV1 =",output)

for i in range(5):
    for j in range(3):
        output[i][j]=UAV2[i][j]+UAV2[i][j+3]
print("UAV2 =",output)

for i in range(5):
    for j in range(3):
        output[i][j]=UAV3[i][j]+UAV3[i][j+3]
print("UAV3 =",output)

for i in range(5):
    for j in range(3):
        output[i][j]=UAV4[i][j]+UAV4[i][j+3]
print("UAV4 =",output)

for i in range(5):
    for j in range(3):
        output[i][j]=UAV5[i][j]+UAV5[i][j+3]
print("UAV5 =",output)