const express = require("express");
const createServer = require("http").createServer;
const app = express();
const fs = require("fs");
const path = require("path");

app.use(express.json());
const server = createServer(app, {});
app.use(express.static('public'));

app.get("/", (req,res)=>{
    let usersPath = path.join(process.cwd(), './view/index.html');
    let fileContents = fs.readFileSync(usersPath, 'utf-8');
    
    // Kirim isi file sebagai respons
    res.send(fileContents);
});

app.get("/home", (req,res)=>{
    let usersPath = path.join(process.cwd(), './view/home.html');
    let fileContents = fs.readFileSync(usersPath, 'utf-8');
  
    // Kirim isi file sebagai respons
    res.send(fileContents);
});

app.get("/data", (req,res)=>{
    res.send({
        hdg : "249.0°",
        sog : "8.7",
        cog : "251.7",
        day : "SUN",
        date : "24/07/2024",
        gps : "1233"
    });
});

app.listen(3000,'0.0.0.0', ()=>{console.log("Listening");});
