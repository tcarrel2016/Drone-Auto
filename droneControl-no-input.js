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
    ? Fix the "max call stack size exceeded" error: don't use recursion for finding blobs anymore.
 
*/

//COLOR KEY:
/*
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
var altitude = 0.000
var orientation = [0.000,0.000,0.000]
var origin = [0,0,0]
var velocity = [0.000,0.000,0.000]      //not working

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
var previousX = 0
var previousY = 0
var previousZ = 0

var color1 = [240,172,110]
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
          //controlFlight()
          count++
          })

//if (count < 30) {
//    client.takeoff()
//}

//.................................................................... DECLARATION

function getMotionData(navdata) {
    if (count > 20) {
        if (count < 30) {
            origin[0] = navdata.demo.rotation.roll
            origin[1] = navdata.demo.rotation.pitch
            origin[2] = navdata.demo.rotation.yaw
        }
        else {
            orientation[0] = navdata.demo.rotation.roll
            orientation[1] = navdata.demo.rotation.pitch
            orientation[2] = navdata.demo.rotation.yaw
            velocity[0] = navdata.demo.velocity.x
            velocity[1] = navdata.demo.velocity.y
            velocity[2] = navdata.demo.velocity.z
            
//            console.log("   orientation: " + orientation)
//            console.log("       velocity: " + velocity)
        }
    }
}

function controlFlight() {
    console.log(String(count))
    
    if (count < 300) {
        client.stop()
        
        if (pathA > -1 && pathX > -1 && pathY > -1) {
            var distance = math.sqrt(math.pow(pathX,2) + math.pow(pathY,2))
            var angleV = math.pi * 1.5
            angleV = pathA - angleV

            if (distance > 360/4) { //CENTER OVER THE PATH OR MOVE FORWARD
                var xV = pathX - (640*0.5)
                var yV = pathY - (320*0.5)
                
                xV /= math.abs(xV)
                yV /= math.abs(yV)
                
                xV *= 0.1
                yV *= 0.1
                
                if (xV > 0) {
                    client.right(xV)
                    console.log("RIGHT")
                }
                else {
                    client.left(math.abs(xV))
                    console.log("LEFT")
                }
                if (yV > 0) {
                    client.back(yV)
                    console.log("BACK")
                }
                else {
                    client.front(math.abs(yV))
                    console.log("FRONT")
                }
            }
            else if (math.abs(angleV) > (math.pi*0.1)) {     //ROTATE
                if (math.abs(angleV) < (math.pi*0.5)) {
                    if (angleV > 0) {
                        client.clockwise(0.2)
                        console.log("CLOCK")
                    }
                    else if (angleV < 0) {
                        client.counterClockwise(0.2)
                        console.log("COUNTER")
                    }
                }
                else {
                    console.log("PATH IS PERPENDICULAR")
                }
            }
            else {  //HOVER
                if (orientation[0] < origin[0]-4) {
                    client.right(0.1)
                }
                else if (orientation[0] > origin[0]+4) {
                    client.left(0.1)
                }
                if (orientation[1] < origin[1]-4) {
                    client.back(0.1)
                }
                else if (orientation[1] >origin[1]+4) {
                    client.front(0.1)
                }
                console.log("HOVER")
            }
        }
        else {  //HOVER
            if (orientation[0] < origin[0]-4) {
                client.right(0.1)
            }
            else if (orientation[0] > origin[0]+4) {
                client.left(0.1)
            }
            if (orientation[1] < origin[1]-4) {
                client.back(0.1)
            }
            else if (orientation[1] > origin[1]+4) {
                client.front(0.1)
            }
            console.log("HOVER")
        }
    }
    else {
        if ((count > 300 || count == 300) && count < 310) {
            client.stop()
            client.land()
        }
    }
}

function processImage(input) {
    pngImage = input
    jimp.read(pngImage, function(err, image) {
              if (err) throw err
              image = thresholdImage(image)
              //findBlobs(image)
              findBlobsNoRecursion(image)
              console.log("BLOBS.# = " + blobsFound.blobs.length)
              analyzeBlobs()
              var line = findLines()
              var marker = findJunctions()
              
              if (marker[0] > -1 && marker[1] > -1) {
                image.setPixelColor(jimp.rgbaToInt(255,0,0,255),marker[0],marker[1])
                for (var i=0; i<marker[2]; i++) {
                  if (marker[0] + i + 1 < image.bitmap.width) {
                    image.setPixelColor(jimp.rgbaToInt(255,0,0,255),marker[0]+i+1,marker[1])
                  }
                }
              }
              else {
                //console.log("NO JUNCTIONS")
              }
              
              if (line[0] > -1 && line[1] > -1 && line[2] > -1) {
                var vectorX = math.cos(line[2]) * 1
                var vectorY = math.sin(line[2]) * 1
              
                for (var i=1; i<20; i++) {
                    image.setPixelColor(jimp.rgbaToInt(0,100,255,255),line[0] + math.round(vectorX*i),line[1] + math.round(vectorY*i))
                }
                image.setPixelColor(jimp.rgbaToInt(255,255,0,255),line[0] + math.round(vectorX*20),line[1] + math.round(vectorY*20))
                console.log("path: " + line[2])
              }
              else {
                console.log("NO LINES")
              }
              
              markBlobs(image)
              
              //if (count % 2 == 0) {
                image.write("./droneControlOutput/img_" + count + ".png")
              //}
              
              markerX = marker[0]
              markerY = marker[1]
              markerR = marker[2]
              pathX = line[0]
              pathY = line[1]
              pathA = line[2]
              })
}

function thresholdImage(image) {
    for (var y = 0; y < image.bitmap.height - skipSize; y += skipSize) {
        for (var x = 0; x < image.bitmap.width - skipSize; x += skipSize) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y))
            if (color.r / color.b > (color1[0]/color1[2]) - 0.6 && color.r / color.b < (color1[0]/color1[2]) + 1 && color.r / color.g > (color1[0]/color1[1]) - 0.6 && color.r / color.g < (color1[0]/color1[1]) + 1) {     //ORANGE
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

function findBlobsNoRecursion(image) {
    blobsFound.blobs = []
    var pixNums = [0,0]
    
    for (var y = 0; y < image.bitmap.height - skipSize; y += skipSize) {
        for (var x = 0; x < image.bitmap.width - skipSize; x += skipSize) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y))
            var inBlob = false
            
            if (color.b > 0) {                                      //type1 = 255, type2 = 100
                pixNums[0]++
                
                for (var i=0; i<blobsFound.blobs.length; i++) {
                    for (var j=0; j<blobsFound.blobs[i].links.length; j++) {
                        if (blobsFound.blobs[i].links[j].x == x && blobsFound.blobs[i].links[j].y == y) {
                            inBlob = true
                        }
                        if (inBlob) {
                            break
                        }
                    }
                }
            }
            else {
                pixNums[1]++
            }
            
            if (!inBlob) {
                console.log("NEW POSSIBLE BLOB FOUND...")
                
                var edges = []
                var links = []
                var news = []
                
                news.concat(new Link(x,y))  //PROBLEM HERE?
                
                while (news.length > 0) {
                    var len = news.length
                    console.log("...len = " + len + "...")
                    
                    for (var i = len-1; i > -1; i--) {
                        if (y-skipSize > 0 && y+skipSize < image.bitmap.height && x-skipSize > 0 && x+skipSize < image.bitmap.width) {
                            var x = news[i].x
                            var y = news[i].y
                            
                            color = jimp.intToRGBA(image.getPixelColor(x,y-skipSize))
                            if (color.b == 255) {
                                news.concat(new Link(x,y-skipSize))
                            }
                            
                            color = jimp.intToRGBA(image.getPixelColor(x,y+skipSize))
                            if (color.b == 255) {
                                news.concat(new Link(x,y+skipSize))
                            }
                            
                            color = jimp.intToRGBA(image.getPixelColor(x-skipSize,y))
                            if (color.b == 255) {
                                news.concat(new Link(x-skipSize,y))
                            }
                            
                            color = jimp.intToRGBA(image.getPixelColor(x+skipSize,y))
                            if (color.b == 255) {
                                news.concat(new Link(x+skipSize,y))
                            }
                        }
                        
                        if (isEdge(image,x,y,1)) {
                            edges.concat(new Link(x,y))
                        }
                        
                        links.concat(news[i])
                        news.splice(i,1)
                        console.log("...NEW ANALYZED...")
                    }
                }
                
                if (links.length > 5) {
                    console.log("...BLOB ADDED")
                    blobsFound.addBlob(1)
                    blobsFound.blobs[blobsFound.blobs.length-1].links = links
                    blobsFound.blobs[blobsFound.blobs.length-1].edges = edges
                }
            }
        }
    }
    
    console.log("+: " + pixNums[0] + ", -: " + pixNums[1])
}

function isEdge(image, x, y, type) {
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

function findBlobs(image) {
    blobsFound.blobs = []
    
    for (var y = 0; y < image.bitmap.height - skipSize; y += skipSize) {
        for (var x = 0; x < image.bitmap.width - skipSize; x += skipSize) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y))
            
            if (color.b > 0) {
                blobsFound.addBlob(1)
                
                if (color.b == 255) {
                    checkLinks(image, x, y, 0, 1)
                }
                else if (color.b == 100) {
                    checkLinks(image, x, y, 0, 2)
                }
            }
        }
    }
}

function checkLinks(image, x, y, direction, type) {
    var inBlob = false
    
    for (var i=0; i<blobsFound.blobs.length; i++) {
        for (var j=0; j<blobsFound.blobs[i].links.length; j++) {
            if (blobsFound.blobs[i].links[j].x == x && blobsFound.blobs[i].links[j].y == y) {
                inBlob = true
            }
            if (inBlob) {
                break
            }
        }
    }
    
    if(inBlob) {
        if (blobsFound.blobs[blobsFound.blobs.length-1].links.length == 0) {
            blobsFound.blobs.pop()
        }
    }
    else {
        if (blobsFound.blobs[blobsFound.blobs.length-1].links.length < 1000) {
            blobsFound.blobs[blobsFound.blobs.length-1].addLink(x,y)
            checkEdge(image,x,y,type)
        
            if (direction != 0 && y-skipSize > 0) {
                var color = jimp.intToRGBA(image.getPixelColor(x,y-skipSize))
                
                if (color.b == 255) {
                    checkLinks(image, x, y-skipSize, 2, type)
                }
            }
            if (direction != 1 && x+skipSize < image.bitmap.width) {
                var color = jimp.intToRGBA(image.getPixelColor(x+skipSize,y))
                
                if (color.b == 255) {
                    checkLinks(image, x+skipSize, y, 3, type)
                }
            }
            if (direction != 2 && y+skipSize < image.bitmap.height) {
                var color = jimp.intToRGBA(image.getPixelColor(x,y+skipSize))
                
                if (color.b == 255) {
                    checkLinks(image, x, y+skipSize, 0, type)
                }
            }
            if (direction != 3 && x-skipSize > 0) {
                var color = jimp.intToRGBA(image.getPixelColor(x-skipSize,y))
                
                if (color.b == 255) {
                    checkLinks(image, x-skipSize, y, 1, type)
                }
            }
        }
    }
}

function checkEdge(image, x, y, type) {
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
    
    if (neighbors > 1 && neighbors < 7 && blobsFound.blobs[blobsFound.blobs.length-1].edges.length < 300) {
        blobsFound.blobs[blobsFound.blobs.length-1].addEdge(x,y)
    }
}

function markBlobs(image) {
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

function analyzeBlobs() {
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

function findLines() {
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

function findJunctions() {
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