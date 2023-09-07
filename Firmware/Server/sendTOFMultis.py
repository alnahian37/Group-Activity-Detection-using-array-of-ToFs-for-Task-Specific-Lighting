import telnetlib
import time
import numpy as np

scale = 1

rgbSpot = [255, 255, 255]
rgbEdge = [255, 255, 255]

def loopScenes():
    while(True):
        array1 = transformMultisUnityToTroffer(arrayBlackboard1)
        array2 = transformMultisUnityToTroffer(arrayBlackboard2)

        array2.insert(0, rgbEdge[2])
        array2.insert(0, rgbEdge[1])
        array2.insert(0, rgbEdge[0])

        array2.insert(0, rgbSpot[2])
        array2.insert(0, rgbSpot[1])
        array2.insert(0, rgbSpot[0])

        array1.insert(0, smoother)
        array2.insert(0, smoother)

        # Send the array to Arduino as a string
        # tn1Right.write(str(array1).encode('utf-8'))
        # tn1Left.write(str(array1).encode('utf-8'))
        tn2Right.write(str(array2).encode('utf-8'))
        tn2Left.write(str(array2).encode('utf-8'))

        time.sleep(3)

        array1 = transformMultisUnityToTroffer(arraySetForget1)
        array2 = transformMultisUnityToTroffer(arraySetForget2)

        array2.insert(0, rgbEdge[2])
        array2.insert(0, rgbEdge[1])
        array2.insert(0, rgbEdge[0])

        array2.insert(0, rgbSpot[2])
        array2.insert(0, rgbSpot[1])
        array2.insert(0, rgbSpot[0])
        
        array1.insert(0, smoother)
        array2.insert(0, smoother)

        # Send the array to Arduino as a string
        # tn1Right.write(str(array1).encode('utf-8'))
        # tn1Left.write(str(array1).encode('utf-8'))
        tn2Right.write(str(array2).encode('utf-8'))
        tn2Left.write(str(array2).encode('utf-8'))

        time.sleep(3)

        array1 = transformMultisUnityToTroffer(arrayWestWall1)
        array2 = transformMultisUnityToTroffer(arrayWestWall2)

        array2.insert(0, rgbEdge[2])
        array2.insert(0, rgbEdge[1])
        array2.insert(0, rgbEdge[0])

        array2.insert(0, rgbSpot[2])
        array2.insert(0, rgbSpot[1])
        array2.insert(0, rgbSpot[0])

        array1.insert(0, smoother)
        array2.insert(0, smoother)

        # Send the array to Arduino as a string
        # tn1Right.write(str(array1).encode('utf-8'))
        # tn1Left.write(str(array1).encode('utf-8'))
        tn2Right.write(str(array2).encode('utf-8'))
        tn2Left.write(str(array2).encode('utf-8'))

        time.sleep(3)

        array1 = transformMultisUnityToTroffer(arrayTableTRF1)
        array2 = transformMultisUnityToTroffer(arrayTableTRF2)

        array2.insert(0, rgbEdge[2])
        array2.insert(0, rgbEdge[1])
        array2.insert(0, rgbEdge[0])

        array2.insert(0, rgbSpot[2])
        array2.insert(0, rgbSpot[1])
        array2.insert(0, rgbSpot[0])

        array1.insert(0, smoother)
        array2.insert(0, smoother)

        # Send the array to Arduino as a string
        # tn1Right.write(str(array1).encode('utf-8'))
        # tn1Left.write(str(array1).encode('utf-8'))
        tn2Right.write(str(array2).encode('utf-8'))
        tn2Left.write(str(array2).encode('utf-8'))

        time.sleep(6)

def transformMultisUnityToTroffer(listMultis):
    # Scale the multis to 0-4095 and round to integers
    listIntMultis = [round((4095*i*scale), 0) for i in listMultis]
    array = np.array(listIntMultis)
    # Splitting the array into two parts
    edgeMultis = array[:4]
    edgeMultisTransform = np.array([edgeMultis[2], edgeMultis[0], edgeMultis[1], edgeMultis[3]])
    # edgeMultisTransform = np.array([0,0,0,0])
    print(edgeMultisTransform)
    pixelMultis = array[4:]
    pixelMatrix = pixelMultis.reshape((7, 7))
    pixelMatrix = np.flip(pixelMatrix, axis=0)  # Flip the pixel matrix upside down
    print(pixelMatrix)
    mergedMultis = np.concatenate((edgeMultisTransform, pixelMatrix.flatten()), axis=0)
    mergedMultisList = mergedMultis.tolist()  # Flatten the matrix to a list
    return mergedMultisList

# Telnet server IP address and port number
HOST = ["192.168.0.80", "192.168.0.81", "192.168.0.61", "192.168.0.57"]
PORT = 23  # Telnet server port number

# Connect to Telnet servers
# tn1Right = telnetlib.Telnet(HOST[0], PORT)
# tn1Left = telnetlib.Telnet(HOST[1], PORT)
tn2Right = telnetlib.Telnet(HOST[2], PORT)
tn2Left = telnetlib.Telnet(HOST[3], PORT)

start_time = time.perf_counter()  # Record the start time

smoother = 200  # To control how smooth the trasnsition is

arrayBlackboard1 = [0.04698177834015674,2.168404344971009e-19,0.0,0.43003065168039434,0.0,0.0,0.0,0.0,0.0,1.1102230246251565e-16,-2.220446049250313e-16,0.0,0.0,0.0,0.0,0.0,0.0,5.551115123125783e-17,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.1102230246251565e-16,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.1102230246251565e-16,0.0,0.0,0.0,0.0,0.6007226791140322,0.07972865421762429,0.20142562124701385,-8.673617379884035e-18,0.0,0.0,0.8978672465379832,0.029630949288360145,0.3447596422702034,0.21995169915847085,0.0,0.0]
arrayBlackboard2 = [0.0,0.0,0.0,0.4295250957791621,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.1102230246251565e-16,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.05061579578978903,0.12726663700989133,0.5151868856098223,0.0,0.0,0.0,0.0,0.2961072136481161,0.28550356246407566,0.18718553958980771,0.7489847834464022,0.0]

arraySetForget1 = [0.3170193408820132,0.5896602295845403,0.0,0.13498011252268072,-1.734723475976807e-18,0.49856180821099805,0.0,0.0,1.1102230246251565e-16,0.0,1.0,0.0,0.3441926525821332,0.0,0.0,0.0,0.0,0.7785891163428466,0.0,0.382991213002153,0.0,0.0,0.0,0.0,0.4489916633328366,0.0,0.15863115231078934,0.0,0.0,0.0,0.0,0.6502339205598788,0.0,0.4661031965152431,2.0816681711721685e-17,0.17967862786418945,0.0,0.0,0.8128336613697471,0.11786156226830694,0.3422220200395244,0.11572026946608394,0.013711234136760675,0.0,0.0,3.469446951953614e-18,0.3567275111773847,0.0,0.0,0.0,0.0,5.551115123125783e-17,0.0]
arraySetForget2 = [0.2096443845704729,0.0,0.5331483185146028,0.16988259478248596,1.0,-1.1102230246251565e-16,0.0,1.1102230246251565e-16,0.0,0.31038848156491033,0.0,0.15990823697676837,0.0,0.0,0.0,0.0,0.016010056560364805,0.0,0.5909762179133522,0.0,1.1102230246251565e-16,0.0,0.0,0.3475878485292129,0.0,0.35331718232046894,0.0,0.0,0.0,0.0,0.01396522737045941,0.0,0.7844120589223422,0.0,0.2423408075973112,0.3276653848296086,0.0,0.3461928865830159,0.0,-6.938893903907228e-17,0.0,0.0,0.023921249086023666,0.14243757884844954,0.3791950639412438,0.01192923335379617,0.0,0.0,0.0,0.0,0.0,0.0,0.4347969896848071]

arrayWestWall1 = [0.0,0.0,0.5141281105140175,0.0,0.0,0.0,0.0,0.0,0.0,-1.1102230246251565e-16,0.07086555582875312,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.1102230246251565e-16,0.0,0.0,1.1102230246251565e-16,0.0,0.0,0.0,2.220446049250313e-16,0.0,0.0,0.016293349714350154,-2.220446049250313e-16,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
arrayWestWall2 = [0.0,0.0,0.7108561749846084,0.0,0.0,-1.1102230246251565e-16,0.0,0.0,0.0,1.3877787807814457e-17,0.0,0.0,0.0,0.0,0.0,0.0,0.4865336206656893,0.07131666293980132,0.0,0.0,-1.1102230246251565e-16,0.0,0.0,0.12663455843317584,0.1453631610414208,0.0,0.0,0.0,0.0,0.0,0.7535805947216534,5.204170427930421e-18,0.0,0.0,0.0,0.0,0.0,0.08856333152324222,0.3071940585311371,1.1102230246251565e-16,0.0,0.0,0.0,0.0,0.0,-1.1102230246251565e-16,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

arrayTable1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,-1.1102230246251565e-16,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.1102230246251565e-16,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
arrayTable2 = [0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.1102230246251565e-16,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.044624676730777926,0.0,1.1102230246251565e-16,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

arrayTableTRF1 = [2.1784516020609194e-27,0.07826565468007092,1.3765408759979368e-27,0.00916849392192857,9.659118170106254e-26,1.822849158714278e-24,4.9209471320300815e-28,1.9661322801279487e-26,0.37298023723301765,6.360976142119373e-27,2.3854816416408666e-26,8.258826483025973e-24,6.621661150083889e-05,3.1762513772896123e-27,1.9844454672483611e-26,0.45143202803478155,5.23685800233673e-27,3.6943003158764235e-26,1.3745738364597685e-30,0.06924983486283676,1.3857244050925433e-28,2.717343356818243e-28,0.687594579936755,9.904932842080234e-27,1.0735217394194107e-25,1.1826650969796014e-29,0.010849564575548315,9.664332105236816e-28,1.8062344097315618e-28,1.4594775562390176e-28,3.1591968936485356e-28,4.115817065186345e-26,1.217624518441525e-29,0.14310798351275614,2.7569265143720093e-26,5.12926277919516e-27,6.998258641057628e-27,1.1873954643317657e-27,0.012822797426295551,1.6096744358524558e-27,0.049232972063666856,7.583221133536846e-26,8.564049772009159e-26,3.2101798073345415e-26,4.1904884894564826e-26,1.2649853002227755e-27,2.2354853887775152e-23,1.0078785867434748e-27,1.9629463486869232e-28,2.0908866360545468e-28,9.654941117484746e-28,1.4590806940500598e-27,2.125340404582388e-27]
arrayTableTRF2 = [1.1338067667421642e-27,0.00380523228855305,0.08585274522876679,0.022865102015782394,3.3760652087285995e-26,1.2866449576381492e-27,0.2703278933069989,7.490980279587183e-27,1.6897713472208682e-28,6.488208049831062e-27,1.6213023904062083e-27,2.8270173146753723e-23,1.7265657386957842e-26,0.056316107857367875,4.995197972097473e-26,1.6603702229897543e-27,2.118746502210828e-25,3.079204433579853e-27,6.129732008908431e-26,9.473768147184102e-27,0.5717005290669344,1.1857010787871455e-26,2.025802958788526e-27,0.05607617608314063,1.763267062746177e-28,9.54196941256143e-26,2.3054436629984766e-27,5.340417184127033e-28,6.222514088254128e-28,5.019324727827685e-28,0.04068880595790465,2.887799237576462e-28,6.294755758067751e-20,3.3524595618798273e-26,1.7072584957567857e-27,2.2021508101067882e-27,7.76683564109438e-30,0.11953665212862634,3.466056556242398e-28,2.202179906119284e-27,5.636930983894515e-26,2.9970017564350243e-25,4.552063810683594e-25,0.04782193881885238,0.068191868601349,4.256377400902809e-27,5.843415303773261e-28,9.949451796092094e-28,8.86458369722357e-28,5.7746408481005775e-28,3.3586458952207174e-28,4.060538085980319e-28,0.02977859386015899]

# array = np.zeros(49)

# array[6] = 1
# array[42] = 1
# array[48] = 1

# array1 = transformMultisUnityToTroffer(arrayBlackboard1)
# array2 = transformMultisUnityToTroffer(arrayBlackboard2)

# array1 = transformMultisUnityToTroffer(arraySetForget1)
# array2 = transformMultisUnityToTroffer(arraySetForget2)

# array1 = transformMultisUnityToTroffer(arrayWestWall1)
# array2 = transformMultisUnityToTroffer(arrayWestWall2)

# array1 = transformMultisUnityToTroffer(arrayTable1)
# array2 = transformMultisUnityToTroffer(arrayTable2)

# array1 = transformMultisUnityToTroffer(arrayTableTRF1)
# array2 = transformMultisUnityToTroffer(arrayTableTRF2)

# array1 = (arrayBlackboard1+arrayTable1)/2
# array2 = (arrayBlackboard2+arrayTable2)/2

# array[2] = 2000
array2 = [0.0] * 53
# array2[0] = 1
# array2[1] = 1
# array2[2] = 1
# array2[3] = 1
# array2[4] = 1
array2 = transformMultisUnityToTroffer(array2)

array2.insert(0, rgbEdge[2])
array2.insert(0, rgbEdge[1])
array2.insert(0, rgbEdge[0])

array2.insert(0, rgbSpot[2])
array2.insert(0, rgbSpot[1])
array2.insert(0, rgbSpot[0])

# array1.insert(0, smoother)
array2.insert(0, smoother)

# print(array2)

# Send the array to Arduino as a string
# tn1Right.write(str(array1).encode('utf-8'))
# tn1Left.write(str(array1).encode('utf-8'))
# tn2Right.write(str(array2).encode('utf-8'))
# tn2Left.write(str(array2).encode('utf-8'))

# print(tn2Right.read_all()) # Read all the data from the Telnet server

loopScenes()

end_time = time.perf_counter()  # Record the end time

# Closing all the connections
# tn1Right.close()
# tn1Left.close()
tn2Right.close()
tn2Left.close()

# Calculate the elapsed time in milliseconds
elapsed_time = (end_time - start_time) * 1000

# Print the elapsed time in milliseconds
print("Elapsed time: {:.2f} milliseconds".format(elapsed_time))
