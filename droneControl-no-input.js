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
    • Use Ø(droneDirection-blobDirection) to control Yaw angle
    √ Use bottom camera
    • Incorporate second color for junctions, with original functions
 
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

var client = ardrone.createClient()
var pngImage
var markerX = -1
var markerY = -1
var markerR = -1
var pathA = -1
var erosionFactor = 2
var count = 0
var skipSize = 3
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
    
    console.log(String(count))
    
    if (count < 300 && count > 50) {
        processImage(incoming)
        client.stop()
    
        if (pathA > -1) {
            var angleV = math.pi * 1.5
            angleV = pathA - angleV
    
            if (math.abs(angleV) > (math.pi*0.1)) {
                if (math.abs(angleV) < (math.pi*0.5)) {
                    if (angleV > 0) {
                        client.clockwise(0.2)
                    }
                    else if (angleV < 0) {
                        client.counterClockwise(0.2)
                    }
                }
                else {
                    console.log("PATH TOO FAR!")
                }
            }
        }
    }
    else if (count < 30) {
        client.stop()
        client.down(0.1)
        client.front(0.01)
    }
    else {
        if (count > 300 || count == 300) {
            client.stop()
            client.land()
        }
    }
    
    count++
    })

client.takeoff()


//....................................................................................................

function processImage(input) {
    pngImage = input
    jimp.read(pngImage, function(err, image) {
              if (err) throw err
              image = thresholdImage(image)
              findBlobs(image)
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
                console.log("NO JUNCTIONS")
              }
              
              if (line[0] > -1 && line[1] > -1 && line[2] > -1) {
                var vectorX = math.cos(line[2]) * 1
                var vectorY = math.sin(line[2]) * 1
              
                for (var i=1; i<20; i++) {
                    image.setPixelColor(jimp.rgbaToInt(0,100,255,255),line[0] + math.round(vectorX*i),line[1] + math.round(vectorY*i))
                }
                image.setPixelColor(jimp.rgbaToInt(255,255,0,255),line[0] + math.round(vectorX*20),line[1] + math.round(vectorY*20))
                console.log("PATH: " + line[2])
              }
              else {
                console.log("NO LINES")
              }
              
              markBlobs(image)
              
              if (count % 5 == 0) {
                image.write("./ardroneAutonomousControlOutput/image_" + count + ".png")
              }
              //output = marker
              markerX = marker[0]
              markerY = marker[1]
              markerR = marker[2]
              pathA = line[2]
              })
}

function thresholdImage(image) {
    for (var y = 0; y < image.bitmap.height - skipSize; y += skipSize) {
        for (var x = 0; x < image.bitmap.width - skipSize; x += skipSize) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y))
            if (color.r / color.b > (color1[0]/color1[2]) - 0.6 && color.r / color.b < (color1[0]/color1[2]) + 1 && color.r / color.g > (color1[0]/color1[1]) - 0.6 && color.r / color.g < (color1[0]/color1[1]) + 1) {     //ORANGE, optimized for band room
                image.setPixelColor(jimp.rgbaToInt(255,255,255,255),x,y)
            }
            /*else if (color.r / color.b > (color2[0]/color2[2]) - 0.5 && color.r / color.b < (color2[0]/color2[2]) + 0.5 && color.r / color.g > (color2[0]/color2[1]) - 0.5 && color.r / color.g < (color2[0]/color2[1]) + 0.5) {
                image.setPixelColor(jimp.rgbaToInt(100,100,100,255),x,y)
            }*/
            else {
                image.setPixelColor(jimp.rgbaToInt(0,0,0,255),x,y)
            }
        }
    }
    
    return image
}

function findBlobs(image) {
    blobsFound.blobs = []
    
    for (var y = 0; y < image.bitmap.height - skipSize; y += skipSize) {
        for (var x = 0; x < image.bitmap.width - skipSize; x += skipSize) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y))
            
            if (color.b > 0) {
                blobsFound.addBlob(1)
                checkLinks(image, x, y, 0)
            }
        }
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

function checkLinks(image, x, y, direction) {       //This needs to be changed to take in two arguments, one for each color being searched
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
        blobsFound.blobs[blobsFound.blobs.length-1].addLink(x,y)
        checkEdge(image,x,y)
        
        if (direction != 0 && y-skipSize > 0) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y-skipSize))
            
            if (color.b > 0) {
                checkLinks(image, x, y-skipSize, 2)
            }
        }
        if (direction != 1 && x+skipSize < image.bitmap.width) {
            var color = jimp.intToRGBA(image.getPixelColor(x+skipSize,y))
            
            if (color.b > 0) {
                checkLinks(image, x+skipSize, y, 3)
            }
        }
        if (direction != 2 && y+skipSize < image.bitmap.height) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y+skipSize))
            
            if (color.b > 0) {
                checkLinks(image, x, y+skipSize, 0)
            }
        }
        if (direction != 3 && x-skipSize > 0) {
            var color = jimp.intToRGBA(image.getPixelColor(x-skipSize,y))
            
            if (color.b > 0) {
                checkLinks(image, x-skipSize, y, 1)
            }
        }
    }
}

function checkEdge(image, x, y) {
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
        blobsFound.blobs[blobsFound.blobs.length-1].addEdge(x,y)
    }
}

function analyzeBlobs() {
    for (var i=0; i<blobsFound.blobs.length; i++) {
        blobsFound.blobs[i].calculateCenter()
        
        if (blobsFound.blobs[i].aspects[7] == 1) {
            blobsFound.blobs[i].calculateLinenessDirection()
        }
        else if (blobsFound.blobs[i].aspects[7] == 2) {
            blobsFound.blobs[i].calculateRadiusCircularityDensity()
        }
    }
}

function findLines() {
    var Lnum = 0;
    var bestLine = [2]
    bestLine[0] = 0
    bestLine[1] = 0
    
    for (var i=0; i<blobsFound.blobs.length; i++) {
        if (blobsFound.blobs[i].aspects[7] == 1 && blobsFound.blobs[i].links.length > 5) {
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
        if (blobsFound.blobs[i].aspects[7] == 2 && blobsFound.blobs[i].links.length > 10) {
            Jnum++
            
            var circularity = blobsFound.blobs[i].aspects[3]
            if (circularity < bestCircularity[0]) {
                bestCircularity[0] = circularity
                bestCircularity[1] = i
                bestBlob = i
            }
            
            var density = blobsFound.blobs[i].aspects[4]
            /*if (density > bestDensity[0]) {
                bestDensity[0] = density
                bestDensity[1] = i
            }*/
        }
    }
    
    if (blobsFound.blobs.length > 0 && Jnum > 0) {
        /*if (bestCircularity[1] == bestDensity[1]) {
            bestBlob = bestCircularity[1]
        }
        else {
            var score1 = (blobsFound.blobs[bestCircularity[1]].aspects[4]) - bestCircularity[0]
            var score2 = bestDensity[0] - blobsFound.blobs[bestDensity[1]].aspects[3]
            
            if (score1 > score2) {
                bestBlob = bestCircularity[1]
            }
            else {
                bestBlob = bestDensity[1]
            }
        }*/
        
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
    this.aspects = [8]
    this.aspects[0] = 320   //X
    this.aspects[1] = 200   //Y
    this.aspects[2] = 50    //R adius
    this.aspects[3] = 3     //C ircularity
    this.aspects[4] = 5     //D ensity
    this.aspects[5] = 0     //L ine-ness
    this.aspects[6] = 0     //A ngle
    this.aspects[7] = color //C olor (1=line,2=junction)
}

Blob.prototype.addLink = function(x, y) {
    this.links = this.links.concat(new Link(x, y))
}

Blob.prototype.addEdge = function(x, y) {
    this.edges = this.edges.concat(new Link(x, y))
}

Blob.prototype.calculateCenter = function() {
    var X = 0
    var Y = 0
    
    for (var i=0; i<this.links.length; i++) {
        X += this.links[i].x
        Y += this.links[i].y
    }
    
    X /= this.links.length
    Y /= this.links.length
    
    this.aspects[0] = X
    this.aspects[1] = Y
}

Blob.prototype.calculateRadiusCircularityDensity = function() {
    var L = 0;
    var R = 0;
    var U = 0;
    var D = 0;
    
    for (var i=0; i<this.links.length; i++) {
        if (this.aspects[0] - this.links[i].x > L) {
            L = this.aspects[0] - this.links[i].x
        }
        if (this.links[i].x - this.aspects[0] > R) {
            R = this.links[i].x - this.aspects[0]
        }
        if (this.aspects[1] - this.links[i].y > U) {
            U = this.aspects[1] - this.links[i].y
        }
        if (this.links[i].y - this.aspects[1] > D) {
            D = this.links[i].y - this.aspects[1]
        }
    }
    
    this.aspects[2] = (L+R+U+D) / 4;
    
    if (L-this.aspects[2] < 0) {
        L = -1 * (L-this.aspects[2])
    }
    else {
       L = (L-this.aspects[2])
    }
    if (R-this.aspects[2] < 0) {
        R = -1 * (R-this.aspects[2])
    }
    else {
        R = (R-this.aspects[2])
    }
    if (U-this.aspects[2] < 0) {
        U = -1 * (U-this.aspects[2])
    }
    else {
        U = (U-this.aspects[2])
    }
    if (D-this.aspects[2] < 0) {
        D = -1 * (D-this.aspects[2])
    }
    else {
        D = (D-this.aspects[2])
    }
    
    this.aspects[3] = (L+R+U+D) / 4;
    
    this.aspects[4] = this.links.length / this.aspects[2]
}

Blob.prototype.calculateLinenessDirection = function() {
    var edgeRadii = [this.edges.length]
    var shortest = 700
    var longest = 0
    var arrow = [1,1]
    
    for (var i=0; i<this.edges.length; i++) {
        var edgeRadius = math.sqrt(math.pow(this.edges[i].x - this.aspects[0],2) + math.pow(this.edges[i].y - this.aspects[1],2))
        
        if (edgeRadius < shortest) {
            shortest = edgeRadius
        }
        if (edgeRadius > longest) {
            longest = edgeRadius
            arrow[0] = this.edges[i].x - this.aspects[0]
            arrow[1] = this.edges[i].y - this.aspects[1]
        }
        
        edgeRadii[i] = edgeRadius
    }
    
    var lineness = longest - shortest
    
    this.aspects[5] = lineness
    
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