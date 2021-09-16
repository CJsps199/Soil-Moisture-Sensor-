const char Styles[] PROGMEM = "div.Configuration{width:100%}div.Waiting{position:absolute;top:0;left:0;right:0;bottom:0;background:transparent}div.Configuration div.SystemMessages{position:fixed;z-index:2;top:0;bottom:0;left:0;right:0;display:flex;justify-content:center;align-items:center}div.Configuration div.SystemMessages div.Window{position:relative;background:linear-gradient(#c9c9c9,#eee,#c9c9c9);box-shadow:0 0 1em #333;border-radius:3px;width:90%}div.Configuration div.SystemMessages div.Window div.Button{position:absolute;top:-0.5em;left:-0.65em;width:1.1em;height:1.1em;margin:0;padding:.4em;border-radius:50%;box-shadow:0 0 .2em #616161}div.Configuration div.SystemMessages div.Label{position:relative;padding:2em 2em 1em;overflow-y:auto}@keyframes RotateClock{from{transform:rotate(0)}to{transform:rotate(360deg)}}@keyframes RotateCounterClock{from{transform:rotate(0)}to{transform:rotate(-360deg)}}div.Icon{position:absolute;top:calc(50% - .5em);left:calc(50% - .5em);height:1em;width:1em;background-color:transparent;border-top:.5em solid #EEE;border-right:.5em solid #288e8e;border-bottom:.5em solid #EEE;border-left:.5em solid #288e8e;border-radius:50%;opacity:1 !important;box-shadow:var(--short-box-shadow);animation:RotateClock 2s linear infinite}div.Configuration div.SystemMessages div.Icon div.Icon{display:none;height:.5em;width:.5em;background-color:transparent;border-right:.5em solid #EEE;border-bottom:.5em solid #288e8e;border-left:.5em solid #EEE;border-top:.5em solid #288e8e;border-radius:50%;opacity:1 !important;animation:RotateCounterClock 1s linear infinite}div.PageContent div.TextContent{padding:3% 0 1% 0;text-shadow:-1px 1px #EEE;width:98%;margin:0 1%;font-size:85%;text-align:center}div.PageContent div.ChoiceRow{vertical-align:middle;text-align:left;text-shadow:-1px 1px #EEE;width:100%;text-align:center}div.ContentFooter{width:98%;margin:1% 0;padding:1% 0 1% 0;border-top:1px solid rgba(150,150,150,0.8);color:#888}div.ContentFooter a{color:#888;text-shadow:-1px 1px #EEE}div.FooterLogo{width:100%;height:12em;margin-top:1%;background-position:center;background-size:contain;background-repeat:no-repeat}div.ContentWraper{max-width:70vh}div.NetworksList{width:calc(98% - 4px);text-align:left;margin:1em 1%}div.NetworkRows{width:100%;height:calc(9em - 8px);overflow-y:auto;overflow-x:hidden;transition:height .3s;vertical-align:middle;box-shadow:var(--short-box-shadow);background-color:rgba(50,50,50,0.1)}div.NetworkRowsHidden{height:0}div.NetworkRow{width:100%;height:2em;line-height:2em;cursor:pointer}div.NetworkRow:hover{background-color:rgba(255,255,255,0.2)}div.NetworkRow:hover:active{background-color:rgba(255,255,255,0.4)}div.NetworkRow div.Name{width:calc(98% - 2em - 2em);margin:.5% 1%}div.NetworksList div.Button{padding:0 !important;margin:.5em !important;max-width:5em;height:2em;line-height:2em;width:4em;vertical-align:middle}div.NetworksList div.Messages{text-align:right;width:calc(100% - 1% - 1em - 5em);color:#888;font-style:italic}div.Advanced{width:100%;margin:1em 0 .5em}div.Advanced div.Window{width:100%;max-height:0;transition:max-height .3s;overflow:hidden;margin:1em 0 0 0}div.Advanced div.WindowOpen{max-height:100vh !important}div.Advanced div.Title{width:100%;cursor:pointer;font-weight:bold;font-style:italic;text-align:left;text-indent:1em}div.Advanced div.Title div.Label{vertical-align:middle;text-align:left;text-indent:1em}div.Advanced div.Title div.Arrow{border-left:.6em solid #888;border-right:0;border-bottom:.4em solid transparent;border-top:.4em solid transparent;transition:transform .3s;vertical-align:middle}div.Advanced div.Title div.ArrowDown{transform:rotateZ(90deg)}div.Separator{width:calc(100% - 1em);margin:.5em .5em;border-top:1px solid #AAA;border-bottom:1px solid #CCC}.Button{background:linear-gradient(var(--light-color) 5%,var(--dark-color) 60%);box-shadow:var(--short-box-shadow);text-shadow:none;text-align:center;font-weight:bold;color:white;cursor:pointer;user-select:none;border:1px solid #777;padding:1em 0;margin:1em .5em;width:calc(100% - 1em)}div.Input .Button{padding:.5em 0;margin:0;width:95%}.Button:hover:active{background:linear-gradient(var(--dark-color) 5%,var(--light-color) 60%)}div.DisabledButton{background:linear-gradient(#b9b9b9 5%,#656565 60%)}div.DisabledButton:hover:active{background:linear-gradient(#656565 5%,#b9b9b9 60%)}div.Configuration div.Change{cursor:pointer;padding:.5em 0;margin:1% calc(50% - 7em);font-size:80%;width:14em;color:#DDD;text-shadow:none;user-select:none}div.InputRow{width:100%;margin:.5em 0}div.InputRow div.Name{vertical-align:middle;text-align:right;width:28%;padding:0 2% 0 0}div.InputRow div.Input div.Value{text-align:left;width:98%;padding:0 2% 0 0}div.InputRow div.Input{vertical-align:middle;text-align:left;width:calc(70% - 1.2em)}div.InputRow div.Input input{text-align:center;padding:.3em 0}div.InputRow div.Input input.Large{width:95%;max-width:25em}div.InputRow div.Input input.Short{width:40%}div.Help{width:1em;background-color:#288e8e;border-radius:40%;padding:.1em;box-shadow:0 0 1px #555;color:white;cursor:pointer;text-shadow:-1px 1px #333;text-align:center}div.InputRow div.Help:hover{background-color:#3cacac}div.InputRow div.Input div.Messages{position:absolute;bottom:100%;left:0;right:0;width:100%;font-size:.7em;color:red}div.InputRow div.Input div.Value{font-style:italic}div.InputRow div.Input div.Point{position:absolute;width:5px;height:5px;top:calc(50% - 2.5px);bottom:calc(50% - 2.5px);border-radius:50%;background-color:#F00;box-shadow:0 0 3px #272}div.InputRow div.Input div.Label{text-indent:10px}div.SignalIcon{--swidth:2em;--sheight:1em;height:var(--sheight);width:var(--swidth)}div.SignalIcon div.Bar{position:absolute;width:.3em;bottom:0;background-color:grey;border-top:1px solid #BBB;border-left:1px solid #BBB;border-right:1px solid #777;border-bottom:1px solid #777;box-shadow:0 0 1px #111}div.SignalIcon div.B01{height:.25em;left:.1em}div.SignalIcon div.B02{height:.5em;left:.6em}div.SignalIcon div.B03{height:.75em;left:1.1em}div.SignalIcon div.B04{height:1em;left:1.6em}div.PageContent div.Tabs{vertical-align:middle;text-shadow:-1px 1px #EEE;width:100%;text-align:center;overflow:hidden}div.Tab{width:33.33%;cursor:pointer;background-color:rgba(0,0,0,0.2);padding:2% 0}div.Selected{background-color:transparent}div.T00{border-radius:5px 0 0 0}div.SwitchButton{width:10em;height:3em;vertical-align:middle;cursor:pointer}div.SwitchButton div.Background{position:absolute;top:38%;bottom:38%;left:10%;right:10%;background-color:#f6f6f6;border-radius:3px;box-shadow:inset 0 0 4px #777;cursor:pointer}div.SwitchButton div.Circle{position:absolute;top:.5em;bottom:.5em;background-color:red;cursor:pointer;border-radius:50%;box-shadow:0 0 5px #777;transition:left .2s,right .2s}div.SwitchButton div.CircleON{right:5%;left:calc(95% - 2em);background-image:radial-gradient(circle,yellow,#f06d06)}div.SwitchButton div.CircleOFF{left:5%;right:calc(95% - 2em);background-image:radial-gradient(circle,#cacaca,#847a73)}div.SwitchButton div.Waiting{left:calc(50% - 1em);right:calc(50% - 1em);background-image:radial-gradient(circle,#cacaca,#847a73)}body{margin:0;padding:0;color:#555;background-color:#ebebda;--short-box-shadow:0 0 3px #111;--light-color:#288e8e;--dark-color:#194b4b}div{position:relative;display:inline-block}input{text-align:center}h1{font-size:5vw;text-align:center}div.header{width:100%;text-align:center;background:linear-gradient(#145a5a,#288e8e);box-shadow:0 0 4px #222}.logo_colour{color:#cc9514}div.header div.logo div.title{color:#555;font-size:1.5em;padding:1% 0;font-family:arial,sans-serif;color:white;font-style:italic}div.PageContent{width:calc(98% - 2px);margin:1%;text-align:center;background-color:#BBB;border-radius:5px;border:1px solid #CCC}div.PageContent div.ContentMarc{padding:0;color:#555;width:100%;border-radius:5px;font-family:Arial,Helvetica,Sans-serif;background:linear-gradient(#c9c9c9,#eee,#c9c9c9);box-shadow:0 0 5px #777}div.Configuration div.WiFi div.Tabs{margin:1em 0;overflow:hidden}div.Configuration div.WiFi div.Tabs div.Tab{width:calc(50% - 8px);border-bottom:1px solid #EEE;margin-top:4px;margin-bottom:-1px}div.Configuration div.WiFi div.Tabs div.T00{border-radius:5px 0 0 0;margin-left:4px}div.Configuration div.WiFi div.Tabs div.T01{border-radius:0 5px 0 0;margin-right:4px}div.WiFiClient{width:100%}div.Console{display:flex;flex-direction:column-reverse;width:calc(100% - 1em);margin:0 .5em;height:10em;background-color:white;box-shadow:0 0 5px #888;text-align:left;font-family:monospace;font-size:85%;overflow-y:auto}div.Console div.Line{width:99%;margin:.5%}div.Wan{width:100%}div.RFM{width:100%}div.RFM div.InputRow div.Name{width:38%}div.RFM div.InputRow div.Input{width:calc(60% - 1.2em)}div.Boundary input,div.Boundary{font-style:italic}div.Boundary div.Name{font-style:italic}div.Boundary div.Input input.Large{color:#555}div.Pins div.InputRow{width:50%}div.ContentWraper{width:98%;margin:0 1%;max-width:130vh}div.Configuration div.Tab{width:calc(25% - 2px);cursor:pointer;background-color:rgba(0,0,0,0.2);padding:2% 0}div.Configuration div.Selected{background-color:transparent;font-weight:bold;border-top:1px solid #EEE;border-right:1px solid #CCC;border-left:1px solid #EEE;border-bottom:1px solid transparent;box-shadow:0 0 4px #7d7d7d}div.Configuration div.T00{border-radius:5px 0 0 0}div.Configuration div.T01,div.Configuration div.T02{border-radius:none}div.Configuration div.T03{border-radius:0 5px 0 0}div.Configuration div.Title{width:100%;text-align:left;margin:3% 0 .5%;font-weight:bold}div.LoRaWANGateway{text-align:initial}div.System{width:100%}div.System div.FullVersion{font-family:monospace;font-size:90%}div.ESPS{width:calc(100% - 3px);margin:1% 0;padding:3px}div.ESPS div.UpgradeButton{display:none;width:100%;max-width:15em;padding:.5em 0;margin:.25em 0 .5em}div.ESPS div.FullVersion,div.ESPS div.FullVersion div.Input input{width:calc(100% - 6px);max-width:initial;text-align:center}div.ESPS div.FullVersion div.Name{display:none}div.ESPS div.FullVersion div.Input{width:100%;margin:initial;padding:initial}div.ESPS div.FullVersion div.Input input{background:transparent;border-color:transparent}";