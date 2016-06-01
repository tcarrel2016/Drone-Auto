//ardroneAutonomousControl.js
//image = 640x360
//Blob detection
//Horizontal tracking
//Vertical tracking
//Marks Radius

/* AGENDA

    √ Find blobs
    √ Record edge links
    √ Test bottom camera
    √ Test if edge link detection is done accurately by marking them    NOTE: I'm wondering if links should store an edge? var, if edge finding is asynchronous at all.
    √ Fix glitches with blob detecting
    √   (skipping blobs)
    √   (green on bottom and right borders of the image)
    √ Record radii from center to edge links
    √ Record max-min radial difference
    √ Find blob with largest difference (not average difference)
    √ Get blob line
    √ Find blob line direction
    √ Mark path
    √ Test + fix path marking
    √ Use Ø(droneDirection-blobDirection) to control Yaw angle
    √ Use bottom camera
    • Incorporate second color for junctions, with original functions
    √ Try getting navdata for its orientation to control more accurately it's drift
    √ Figure out how to read navdata (it's not a straight string...)
    √ Use edge pixels when finding junctions and clean up analyzeBlobs()
    √ Incorporate navdata to help hovering
    √ Fix the "max call stack size exceeded" error: don't use recursion for finding blobs anymore.
    √ Fix new errors with findBlobsNoRecursion(): out-of-bounds[√], infinitely-large-blob[√] = problem: pixels that are already links are added as news.
    √ Look up Hough functions that could possibly find lines and replace findBlobsNoRecursion()
    • Fix drone movement: 
        try not updating line data if no new line is found [x],
        don't do any command other than HOVER 2x in a row [x], 
        allow drone to do same command twice w/ timer [?],
        have path shoulders which help if the drone is lost [ ]
        try sending initial navdata-enabling command to see if altitude and velocity data becomes available [ ]
            > the command is: 
            > client.config('general:navdata_demo', 'FALSE');
 
*/

/* COLOR KEY:
        WHITE:  line marker
        GRAY:   junction marker
        RED:    radius
        BLUE:   center, best path estimation
        YELLOW: path direction head
        GREEN:  edge
 */

var ardrone = require('ar-drone')
var jimp = require('./jimp-master/index.js')
var math = require('./mathjs-master/index.js')  //Comprehensive math library (used for square root, exponents, absolute value, vector math, etc.)

//Navdata
var orientation = [0.000,0.000,0.000]
var origin = [0,0,0]

var client = ardrone.createClient()
var pngImage   //640*360
var markerX = -1
var markerY = -1
var markerR = -1
var pathX = -1
var pathY = -1
var pathA = -1
var erosionFactor = 2
var count = 0
var skipSize = 10
var command = [0,0]     //0,1,2,3,4
var pCommand = [0,0]    //0,1,2,3,4 = HOVER,UP,RIGHT,DOWN,LEFT
var pCommandTimer = [0,0];  //counts how long the drone has been trying the same command
var timeOffCourse = 0;
var color1 = [240,100,100]
var color2 = [240,172,110]

var blobsFound = new BlobLibrary()

client.config("video:video_channel", 1)

var pngStream = client.getPngStream()

pngStream
.on("error", console.log)
.on("data", function(incoming) {
    processImage(incoming)
    })

client.on("navdata", function(navdata) {
          getMotionData(navdata)
          
          if (pCommand[0] == command[0]) {
            pCommandTimer[0]++
          }
          else {
            pCommandTimer[0] = 0
          }
          if (pCommandTimer[0] > 50) {
            pCommand[0] = 0
          }
          else {
            pCommand[0] = 0
          }
          
          if (pCommand[1] == command[1]) {
            pCommandTimer[1]++
          }
          else {
            pCommandTimer[1] = 0
          }
          if (pCommandTimer[1] > 45) {
            pCommand[1] = 0
          }
          else {
            pCommand[1] = 0
          }
          
          controlFlight()
          count++
          })

if (count < 30) {
    client.takeoff()
}

//.................................................................... DECLARATION

function getMotionData(navdata) {                   //I wanted to stabilize the drone by countering it's lean
    if (count > 10) {
        if (count < 30) {
            origin[0] = navdata.demo.rotation.roll
            origin[1] = navdata.demo.rotation.pitch
            origin[2] = navdata.demo.rotation.yaw
        }
        else {
            orientation[0] = navdata.demo.rotation.roll
            orientation[1] = navdata.demo.rotation.pitch
            orientation[2] = navdata.demo.rotation.yaw
        }
    }
}

function controlFlight() {                         //Control drone based on given path (X,Y,A)
    if (count < 500 && count > 50) {
        if (pathA > -1 && pathX > -1 && pathY > -1) {
            var distance = math.sqrt(math.pow(pathX-(640*0.5),2) + math.pow(pathY-(320*0.5),2))
            var angleV = math.pi * 1.5
            angleV = pathA - angleV
            
            if (distance > 320/3) {                                                                                 //CENTER OVER THE PATH OR MOVE FORWARD
                timeOffCourse++;
                var xMore = false;
                
                var xV = pathX - (640*0.5)
                var yV = pathY - (320*0.5)
                
                if (math.abs(xV) > math.abs(yV)) {
                    xMore = true;
                }
                
                xV /= math.abs(xV)
                yV /= math.abs(yV)
                
                if ((timeOffCourse*0.001) < 0.04) {
                    xV *= 0.05 - (timeOffCourse*0.0005)
                    yV *= 0.05 - (timeOffCourse*0.0005)
                }
                else {
                    xV *= 0.005; //0.01
                    yV *= 0.005;
                }
                
                if (xV > 0.0) {
                    command[0] = 2
                }
                else if (xV < 0.0) {
                    command[0] = 4
                }
                if (yV > 0.0) {
                    command[1] = 3
                }
                else if (yV < 0.0) {
                    command[1] = 1
                }
                
                client.stop()
                if ((pCommand[1] == 0 || pCommand[1] != command[1]) && !xMore) {
                    if (command[1] == 1) {
                        client.front(math.abs(yV))
                        console.log("FRONT")
                    }
                    else if (command[1] == 3) {
                        client.back(math.abs(yV))
                        console.log("BACK")
                    }
                }
                if ((pCommand[0] == 0 || pCommand[0] != command[0]) && xMore) {
                    if (command[0] == 2) {
                        client.right(math.abs(xV))
                        console.log("RIGHT")
                    }
                    else if (command[0] == 4) {
                        client.left(math.abs(xV*1.5))
                        console.log("LEFT")
                    }
                }
            }
            else {
                timeOffCourse = 0;
                
                if (distance < 320/3 && math.abs(angleV) > 0/*(math.pi*0.1)*/) {     //ROTATE
                    client.stop()
                    if (math.abs(angleV) < (math.pi*0.5)) {
                        if (angleV > 0) {
                            client.clockwise(0.1)
                            console.log("CLOCK")
                        }
                        else if (angleV < 0) {
                            client.counterClockwise(0.1)
                            console.log("COUNTER")
                        }
                    }
                    else {
                        console.log("PATH IS PERPENDICULAR")
                    }
                }
                if (distance < 320/3) {  //HOVER
    //                if (orientation[0] < origin[0]-4) {
    //                    client.right(0.08)
    //                }
    //                else if (orientation[0] > origin[0]+4) {
    //                    client.left(0.08)
    //                }
    //                if (orientation[1] < origin[1]-4) {
    //                    client.back(0.08)
    //                }
    //                else if (orientation[1] >origin[1]+4) {
    //                    client.front(0.08)
    //                }
                    client.stop()
                    client.front(0.02);
                    command = [0,0]
                    console.log("PATH FOUND :)")
                }
            }
        }
        else {                                                                                                      //HOVER
//            if (orientation[0] < origin[0]-4) {
//                client.right(0.08)
//            }
//            else if (orientation[0] > origin[0]+4) {
//                client.left(0.08)
//            }
//            if (orientation[1] < origin[1]-4) {
//                client.back(0.08)
//            }
//            else if (orientation[1] > origin[1]+4) {
//                client.front(0.08)
//            }
            command = [0,0]
            console.log("LOST :(")
        }
    }
    else {
        if ((count > 500 || count == 500) && count < 510) {
            client.stop()
            client.land()
        }
    }
}

function processImage(input) {                     //Find path and junction in image
    pngImage = input
    jimp.read(pngImage, function(err, image) {
              if (err) throw err
              image = thresholdImage(image)
              findBlobsNoRecursion(image)
              analyzeBlobs()
              var line = findLines()
//              var marker = findJunctions()
//              
//              if (marker[0] > -1 && marker[1] > -1) {
//                image.setPixelColor(jimp.rgbaToInt(255,0,0,255),marker[0],marker[1])
//                for (var i=0; i<marker[2]; i++) {
//                  if (marker[0] + i + 1 < image.bitmap.width) {
//                    image.setPixelColor(jimp.rgbaToInt(255,0,0,255),marker[0]+i+1,marker[1])
//                  }
//                }
//              }
//              else {
//                //console.log("NO JUNCTIONS")
//              }
              
              if (line[0] > -1 && line[1] > -1 && line[2] > -1) {
                var vectorX = math.cos(line[2]) * 1
                var vectorY = math.sin(line[2]) * 1
              
                for (var i=1; i<20; i++) {
                    image.setPixelColor(jimp.rgbaToInt(0,100,255,255),line[0] + math.round(vectorX*i),line[1] + math.round(vectorY*i))
                }
                image.setPixelColor(jimp.rgbaToInt(255,255,0,255),line[0] + math.round(vectorX*20),line[1] + math.round(vectorY*20))
                pathX = line[0]
                pathY = line[1]
                pathA = line[2]
              }
              else {
                //console.log("NO LINES")
              }
              
              markBlobs(image)
              
              //image.write("./droneControlOutput/img_" + count + ".png")
              
//              markerX = marker[0]
//              markerY = marker[1]
//              markerR = marker[2]
              })
}

function thresholdImage(image) {                    //Color thresholding
    for (var y = 0; y < image.bitmap.height - skipSize; y += skipSize) {
        for (var x = 0; x < image.bitmap.width - skipSize; x += skipSize) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y))
            if (color.r / color.b > (color1[0]/color1[2]) - 1.5 && color.r / color.b < (color1[0]/color1[2]) + 2.5 && color.r / color.g > (color1[0]/color1[1]) - 1 && color.r / color.g < (color1[0]/color1[1]) + 2.5) {     //~ORANGE
                image.setPixelColor(jimp.rgbaToInt(255,255,255,255),x,y)
            }
            /*else if (color.r / color.b > (color2[0]/color2[2]) - 0.5 && color.r / color.b < (color2[0]/color2[2]) + 0.5 && color.r / color.g > (color2[0]/color2[1]) - 0.5 && color.r / color.g < (color2[0]/color2[1]) + 0.5) {  //GREEN
                image.setPixelColor(jimp.rgbaToInt(100,100,100,255),x,y)
            }*/
            else {
                image.setPixelColor(jimp.rgbaToInt(0,0,0,255),x,y)
            }
        }
    }
    
    return image
}

function findBlobsNoRecursion(image) {            //Find groups of pixels of the same color
    blobsFound.blobs = []   //clear blobs from previous image
    var pixNums = [0,0]     //just to keep track of how many pixels were kept vs. how many were not after thresholding
    
    for (var startY = 0; startY < image.bitmap.height - skipSize; startY += skipSize) {     //Loop through all pixels (accounting for skipSize) in the image
        for (var startX = 0; startX < image.bitmap.width - skipSize; startX += skipSize) {  
            var color = jimp.intToRGBA(image.getPixelColor(startX,startY))  //Get color of current pixel (startX,startY)
            var inBlob = false
            
            if (color.b > 0) {                                      //**COMMENT NOT FOR MR LIN** type1 = 255, type2 = 100
                pixNums[0]++
                
                for (var i=0; i<blobsFound.blobs.length; i++) {     //Loop through all blobs found so far to check if current pixel has already been used
                    for (var j=0; j<blobsFound.blobs[i].links.length && inBlob == false; j++) {
                        if (blobsFound.blobs[i].links[j].x == startX && blobsFound.blobs[i].links[j].y == startY) {
                            inBlob = true
                        }
                    }
                }
            }
            else {
                pixNums[1]++
            }
            
            if (inBlob == false && color.b > 0) {   //If pixel is within threshold and not already used, then create a new blob
                var edges = []  //A selection of links that will be used to find blob radii outside of findBlobsNoRecursion()
                var links = []  //Points that will make up the new blob
                var news = []   //Points that haven't been checked yet for new neighboring white pixels
                
                news.push(new Link(startX,startY))  //Add first pixel to news
                var iteration=0     //Just for me to see how long it takes for the program to finish the blob
                
                while (news.length > 0) {   //While there are still pixels whose neighbors are not checked...
                    var len = news.length   //Number of pixels which, as of now, aren't checked
                    
                    for (var i = len-1; i > -1; i--) {  //Loop through current news[] pixels from last to first (won't include pixels added to the array later in the process)
                        var x = news[i].x   //store location of new pixel to be checked
                        var y = news[i].y
                        
                        if (y-skipSize > 0 && y+skipSize < image.bitmap.height && x-skipSize > 0 && x+skipSize < image.bitmap.width) {  //make sure new pixel is not at the edge of the image
                            color = jimp.intToRGBA(image.getPixelColor(x,y-skipSize))   //START: check neighbor above
                            if (color.b == 255) {   //if neighbor is white
                                var used = false
                                for (var j=0; j<news.length && used == false; j++) {    //loop through new pixels
                                    if (news[j].x == x && news[j].y == y-skipSize) {    //check if neighbor is already added
                                        used = true
                                    }
                                }
                                
                                if (used == false) {
                                    for (var j=0; j<links.length && used == false; j++) {    //loop through saved pixels (already in blob)
                                        if (links[j].x == x && links[j].y == y-skipSize) {  //check if neighbor is already used
                                            used = true
                                        }
                                    }
                                    if (used == false) {
                                        news.push(new Link(x,y-skipSize))   //add neighbor to news[]
                                    }
                                }
                            }   //END: check neighbor above
                            
                            color = jimp.intToRGBA(image.getPixelColor(x,y+skipSize))   //START: check neighbor below
                            if (color.b == 255) {
                                var used = false
                                for (var j=0; j<news.length && used == false; j++) {
                                    if (news[j].x == x && news[j].y == y+skipSize) {
                                        used = true
                                    }
                                    if (used) {
                                        break
                                    }
                                }
                                
                                if (used == false) {
                                    for (var j=0; j<links.length && used == false; j++) {
                                        if (links[j].x == x && links[j].y == y+skipSize) {
                                            used = true
                                        }
                                    }
                                    if (used == false) {
                                        news.push(new Link(x,y+skipSize))
                                    }
                                }
                            }   //END: check neighbor below
                            
                            color = jimp.intToRGBA(image.getPixelColor(x-skipSize,y))   //START: check neighbor left
                            if (color.b == 255) {
                                var used = false
                                for (var j=0; j<news.length && used == false; j++) {
                                    if (news[j].x == x-skipSize && news[j].y == y) {
                                        used = true
                                    }
                                }

                                if (used == false) {
                                    for (var j=0; j<links.length && used == false; j++) {
                                        if (links[j].x == x-skipSize && links[j].y == y) {
                                            used = true
                                        }
                                    }
                                    if (used == false) {
                                        news.push(new Link(x-skipSize,y))
                                    }
                                }
                            }   //END: check neighbor left
                            
                            color = jimp.intToRGBA(image.getPixelColor(x+skipSize,y))   //START: check neighbor right
                            if (color.b == 255) {
                                var used = false
                                for (var j=0; j<news.length && used == false; j++) {
                                    if (news[j].x == x+skipSize && news[j].y == y) {
                                        used = true
                                    }
                                }
                                
                                if (used == false) {
                                    for (var j=0; j<links.length && used == false; j++) {
                                        if (links[j].x == x+skipSize && links[j].y == y) {
                                            used = true
                                        }
                                    }
                                    if (used == false) {
                                        news.push(new Link(x+skipSize,y))
                                    }
                                }
                            } //END: check neighbor right
                        }
                        
                        if (isEdge(image,x,y,1)) {  //check if new pixel is an edge
                            edges.push(new Link(x,y))   //add new pixel to edges[] (for calculating blob's radii later)
                        }
                        
                        links.push(news[i]) //add this pixel to the new blob
                        news.splice(i,1)    //remove this pixel from news[], as it's now checked
                    }
                    iteration++
                }
                
                if (links.length > 5) { //only add blob if it's size is somewhat significant
                    //console.log("...BLOB ADDED @ " + startX + "," + startY) //print blob's initial point
                    blobsFound.addBlob(1)   //add an empty blob (constructor is not currently important)
                    blobsFound.blobs[blobsFound.blobs.length-1].links = links   //fill blob's links[] array
                    blobsFound.blobs[blobsFound.blobs.length-1].edges = edges   //fill blob's edges[] array
                }
                else {
                    //console.log("BLOB TOO SMALL")
                }
            }
        }
    }
    
    //console.log("+: " + pixNums[0] + ", -: " + pixNums[1])  //not important
}

function isEdge(image, x, y, type) {        //Edges used for finding the radii of a blob
    var neighbors = 0
    var color
    
    if (x+skipSize < image.bitmap.width && y-skipSize > 0) {
        color = jimp.intToRGBA(image.getPixelColor(x+skipSize,y-skipSize))
        if (color.b == 255) {
            neighbors++
        }
    }
    
    if (x+skipSize < image.bitmap.width) {
        color = jimp.intToRGBA(image.getPixelColor(x+skipSize,y))
        if (color.b == 255) {
            neighbors++
        }
    }
    
    if (x+skipSize < image.bitmap.width && y+skipSize < image.bitmap.height) {
        color = jimp.intToRGBA(image.getPixelColor(x+skipSize,y+skipSize))
        if (color.b == 255) {
            neighbors++
        }
    }
    
    if (y+skipSize < image.bitmap.height) {
        color = jimp.intToRGBA(image.getPixelColor(x,y+skipSize))
        if (color.b == 255) {
            neighbors++
        }
    }
    
    if (x-skipSize > 0 && y+skipSize < image.bitmap.height) {
        color = jimp.intToRGBA(image.getPixelColor(x-skipSize,y+skipSize))
        if (color.b == 255) {
            neighbors++
        }
    }
    
    if (x-skipSize > 0) {
        color = jimp.intToRGBA(image.getPixelColor(x-skipSize,y))
        if (color.b == 255) {
            neighbors++
        }
    }
    
    if (x-skipSize >0 && y-skipSize > 0) {
        color = jimp.intToRGBA(image.getPixelColor(x-skipSize,y-skipSize))
        if (color.b == 255) {
            neighbors++
        }
    }
    
    if (y-skipSize > 0) {
        color = jimp.intToRGBA(image.getPixelColor(x,y-skipSize))
        if (color.b == 255) {
            neighbors++
        }
    }
    
    if (neighbors > 1 && neighbors < 7) {
        return true
    }
    else {
        return false
    }
}

function markBlobs(image) {                 //Show where the program found blobs
    for (var i=0; i<blobsFound.blobs.length; i++) {
        if (blobsFound.blobs[i].links.length > 5) {
            var location = [blobsFound.blobs[i].aspects[0],blobsFound.blobs[i].aspects[1]]
            
            image.setPixelColor(jimp.rgbaToInt(0,100,255,255),math.round(location[0]),math.round(location[1]))
            
            for (var j=0; j<blobsFound.blobs[i].edges.length; j++) {
                location = [blobsFound.blobs[i].edges[j].x,blobsFound.blobs[i].edges[j].y]
                image.setPixelColor(jimp.rgbaToInt(0,255,0,255),location[0],location[1])
            }
        }
    }
}

function analyzeBlobs() {                 //Calculate data of a blob
    for (var i=0; i<blobsFound.blobs.length; i++) {
        blobsFound.blobs[i].calculateCenterRadii()
        
        if (blobsFound.blobs[i].aspects[7] == 1) {
            blobsFound.blobs[i].calculateLinearityDirection()
        }
        else if (blobsFound.blobs[i].aspects[7] == 2) {
            blobsFound.blobs[i].calculateCircularity()
        }
    }
}

function findLines() {                  //Use blob data to find most likely path
    var Lnum = 0;
    var bestLine = [2]
    bestLine[0] = 0
    bestLine[1] = 0
    
    for (var i=0; i<blobsFound.blobs.length; i++) {
        if (blobsFound.blobs[i].aspects[7] == 1 && blobsFound.blobs[i].links.length > 10) {
            if (blobsFound.blobs[i].aspects[5] > bestLine[0]) {
                bestLine[0] = blobsFound.blobs[i].aspects[5]
                bestLine[1] = i
            }
            Lnum++
        }
    }
    
    if (blobsFound.blobs.length > 0 && Lnum > 0) {
        var lineHeading = blobsFound.blobs[bestLine[1]].aspects[6]
        var angleDifference = math.abs((math.pi*1.5) - lineHeading)
        
        if (angleDifference > math.pi) {
            angleDifference = (2*math.pi) - angleDifference
        }
        if (angleDifference > 0.5*math.pi) {
            lineHeading += math.pi
        }
        
        if (lineHeading > 2*math.pi) {
            lineHeading -= 2*math.pi
        }
        
        var lineData = [blobsFound.blobs[bestLine[1]].aspects[0],blobsFound.blobs[bestLine[1]].aspects[1],lineHeading]
    }
    else {
        var lineData = [-1,-1,-1]
    }
    
    return lineData
}

function findJunctions() {               //Use blob data to find most likely junction
    var Jnum = 0
    
    var bestCircularity = [2]       //circularity, blob#
    bestCircularity[0] = 20
    bestCircularity[1] = 0
    
    var bestDensity = [2]           //density,     blob#
    bestDensity[0] = 0
    bestDensity[1] = 0
    
    var bestBlob = 0
    
    for (var i=0; i<blobsFound.blobs.length; i++) {
        if (blobsFound.blobs[i].aspects[7] == 2 && blobsFound.blobs[i].links.length > 20) {
            Jnum++
            
            var circularity = blobsFound.blobs[i].aspects[3]
            if (circularity < bestCircularity[0]) {
                bestCircularity[0] = circularity
                bestCircularity[1] = i
                bestBlob = i
            }
            
            var density = blobsFound.blobs[i].aspects[4]    //Not used right now...
        }
    }
    
    if (blobsFound.blobs.length > 0 && Jnum > 0) {
        var junctionData = [blobsFound.blobs[bestBlob].aspects[0],blobsFound.blobs[bestBlob].aspects[1],blobsFound.blobs[bestBlob].aspects[2]]
    }
    else {
        var junctionData =[-1,-1,-1]
    }
    
    return junctionData
}

function BlobLibrary() {
    this.blobs = []
}

BlobLibrary.prototype.addBlob = function(color) {
    this.blobs = this.blobs.concat(new Blob(color))
}

function Blob(color) {
    this.links = []
    this.edges = []
    this.radii = []
    this.aspects = [8]
    this.aspects[0] = 320   //X
    this.aspects[1] = 200   //Y
    this.aspects[2] = 50    //R adius
    this.aspects[3] = 3     //C ircularity
    this.aspects[4] = 5     //D ensity
    this.aspects[5] = 0     //L inearity
    this.aspects[6] = 0     //A ngle
    this.aspects[7] = color //C olor (1=line,2=junction)
}

Blob.prototype.addLink = function(x, y) {
    this.links = this.links.concat(new Link(x, y))
}

Blob.prototype.addEdge = function(x, y) {
    this.edges = this.edges.concat(new Link(x, y))
}

Blob.prototype.calculateCenterRadii = function() {
    var X = 0
    var Y = 0
    var edgeRadii = [this.edges.length]
    
    for (var i=0; i<this.links.length; i++) {
        X += this.links[i].x
        Y += this.links[i].y
    }
    
    X /= this.links.length
    Y /= this.links.length
    
    this.aspects[0] = X
    this.aspects[1] = Y
    
    for (var i=0; i<this.edges.length; i++) {
        var edgeRadius = math.sqrt(math.pow(this.edges[i].x - this.aspects[0],2) + math.pow(this.edges[i].y - this.aspects[1],2))
        edgeRadii[i] = edgeRadius
    }
    
    this.radii = edgeRadii
    
    if (this.radii.length > 0) {
        var avgRadius = 0
        
        for (var i=0; i<this.radii.length; i++) {
            avgRadius += this.radii[i]
        }
        avgRadius /= this.radii.length
        
        this.aspects[2] = avgRadius
    }
}

Blob.prototype.calculateCircularity = function() {
    if (this.radii.length > 0) {
        var avgDifference = 0
        
        for (var i=0; i<this.radii.length; i++) {
            avgDifference += (this.radii[i] - this.aspects[2])
        }
        avgDifference /= this.radii.length
        
        this.aspects[3] = avgDifference
    }
    
    this.aspects[4] = this.links.length / this.aspects[2]
}

Blob.prototype.calculateLinearityDirection = function() {
    var shortest = 700
    var longest = 0
    var arrow = [1,1]
    
    for (var i=0; i<this.radii.length; i++) {
        var edgeRadius = this.radii[i]
        
        if (edgeRadius < shortest) {
            shortest = edgeRadius
        }
        if (edgeRadius > longest) {
            longest = edgeRadius
            arrow[0] = this.edges[i].x - this.aspects[0]
            arrow[1] = this.edges[i].y - this.aspects[1]
        }
    }
    
    var linearity = longest - shortest
    
    this.aspects[5] = linearity
    
    var angle = math.atan2(math.abs(arrow[1]), math.abs(arrow[0]))
    
    if (arrow[0] < 0 && arrow[1] > 0) {
        angle = math.pi - angle
    }
    else if (arrow[0] < 0 && arrow[1] < 0) {
        angle = math.pi + angle
    }
    else if (arrow[0] > 0 && arrow[1] < 0) {
        angle = (2*math.pi) - angle
    }
    
    this.aspects[6] = angle
}

function Link(x, y) {
    this.x = x
    this.y = y
}
