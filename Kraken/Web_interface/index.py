#!/usr/bin/python3
# -*- coding: utf-8 -*

print("Content-type: text/html; charset=utf-8\n")

html = """
<!DOCTYPE html><base href="http://raspberrypi.local:8888/"><link rel="stylesheet" type="text/css" media="screen" href="style.css" />

<head>
	<link rel="stylesheet" type="text/css" href="style.css" />
	<script language="javascript" type="text/javascript" src="js/mchp.js"></script>
	<script language="javascript" type="text/javascript" src="js/jquery.js"></script>
    <script language="javascript" type="text/javascript" src="js/jquery.flot.js"></script>
	<script language="javascript" type="text/javascript" src="js/jquery.flot.navigate.js"></script>
	<script language="javascript" type="text/javascript" src="js/jquery.flot.image.js"></script>
</head>

<body>
	<center>
		<form method="get" action="index.htm">
		<div id="status">
			<div id="loading" style="display: none">Erreur de connexion</div>
			<div id="display">
				<table class="table_main">
					<tr>
						<td colspan="2">
							<div class="div_header">
								<h2>G4 Control Panel</h2>
								<h1>Kraken</h1>
							</div>
						</td>
					</tr>
					<tr>
						<td class="td_main_left">
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
							<!--
							<fieldset>
								<legend>Extra 1</legend>
								<table class="table_sub">
									<tr>
										<td>
											<div class="div_plot">
												<div class="div_placeholder" id="placeholder4"></div>
											</div>
										</td>
									</tr>
									<tr>
										<td>
											<div class="div_plot">
												<div class="div_placeholder" id="placeholder5"></div>
											</div>
										</td>
									</tr>
								</table>
							</fieldset>
							-->
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
							<fieldset>
								<legend>Commandes</legend>
								<table class="table_sub">
									<tr>
										<td width="50%" colspan="2">
											<p><input class="input_run" type="submit" name="func" value="RUN"></p>
										</td>
										<td width="50%" colspan="2">
											<p><input class="input_stop" type="submit" name="func" value="STOP"></p>
										</td>
									</tr>
									<tr>
										<td width="25%">
											<p><input class="input_com" type="submit" name="func" value="GOTO"></p>
										</td>
										<td width="25%">
											<p><input class="input_com" type="submit" name="func" value="TOTEM"></p>
										</td>
										<td width="25%">
											<p><input class="input_com" type="submit" name="func" value="BOTTLE"></p>
										</td>
										<td width="25%">
											<p><input class="input_com" type="submit" name="func" value="DROP"></p>
										</td>										
									</tr>
									<tr>
										<td width="25%">
											<p><input class="input_com" type="submit" name="func" value="DYNA"></p>
										</td>
										<td width="25%">
											<p><input class="input_com" type="button" value="JSON" onclick="startData()"></p>
										</td>
										<td width="25%">
											<p><input class="input_com" type="button" value="JSOFF" onclick="stopData()"></p>
										</td>
										<td width="25%">
											<p><input class="input_com" type="submit" name="func" value="RESET"></p>
										</td>										
									</tr>
									<tr>
										<td width="25%">
											<p><input class="input_com" type="submit" name="func" value="COM0"></p>
										</td>
										<td width="25%">
											<p><input class="input_com" type="submit" name="func" value="COM1"></p>
										</td>
										<td width="25%">
											<p><input class="input_com" type="submit" name="func" value="COM2"></p>
										</td>
										<td width="25%">
											<p><input class="input_com" type="submit" name="func" value="COM3"></p>
										</td>										
									</tr>
								</table>
							</fieldset>
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
							<fieldset>
								<legend>État</legend>
								<div class="div_state">
									<h1>~state~</h1>
								</div>
							</fieldset>
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
							<fieldset>
								<legend>Status</legend>
								<table class="table_sub">
									<tr>
										<td class="td_label">
											<p>absoluteTime:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="absoluteTime">?</span></p>
										</td>
										<td class="td_label">
											<p>matchTime:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="matchTime">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>localTime:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="localTime">?</span></p>
										</td>
										<td class="td_label">
											<p>time0:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="time0">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>comL:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="comL">?</span></p>
										</td>
										<td class="td_label">
											<p>comR:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="comR">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>sharpFL:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="sharpFL">?</span></p>
										</td>
										<td class="td_label">
											<p>sharpFR:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="sharpFR">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>sharpBL:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="sharpBL">?</span></p>
										</td>
										<td class="td_label">
											<p>sharpBR:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="sharpBR">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>led:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="led">?</span></p>
										</td>
										<td class="td_label">
											<p>dir:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="dir">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>X0:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="X0">?</span></p>
										</td>
										<td class="td_label">
											<p>Y0:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="Y0">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>X1:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="X1">?</span></p>
										</td>
										<td class="td_label">
											<p>Y1:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="Y1">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>X2:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="X2">?</span></p>
										</td>
										<td class="td_label">
											<p>Y2:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="Y2">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>zoneL0:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneL0">?</span></p>
										</td>
										<td class="td_label">
											<p>zoneR0:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneR0">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>zoneL1:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneL1">?</span></p>
										</td>
										<td class="td_label">
											<p>zoneR1:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneR1">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>zoneL2:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneL2">?</span></p>
										</td>
										<td class="td_label">
											<p>zoneR2:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneR2">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>zoneL3:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneL3">?</span></p>
										</td>
										<td class="td_label">
											<p>zoneR3:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneR3">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>zoneL4:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneL4">?</span></p>
										</td>
										<td class="td_label">
											<p>zoneR4:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneR4">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>zoneL5:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneL5">?</span></p>
										</td>
										<td class="td_label">
											<p>zoneR5:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneR5">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>zoneM0:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneM0">?</span></p>
										</td>
										<td class="td_label">
											<p>zoneM1:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="zoneM1">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>debug0:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="debug0">?</span></p>
										</td>
										<td class="td_label">
											<p>debug1:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="debug1">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>debug2:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="debug2">?</span></p>
										</td>
										<td class="td_label">
											<p>debug3:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="debug3">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>dataTime1:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="dataTime1">?</span></p>
										</td>
										<td class="td_label">
											<p>dataTime2:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="dataTime2">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>data11:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data11">?</span></p>
										</td>
										<td class="td_label">
											<p>data12:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data12">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>data21:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data21">?</span></p>
										</td>
										<td class="td_label">
											<p>data22:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data22">?</span></p>
										</td>
									</tr>
									<!--
									<tr>
										<td class="td_label">
											<p>data41:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data41">?</span></p>
										</td>
										<td class="td_label">
											<p>data42:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data42">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>data51:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data51">?</span></p>
										</td>
										<td class="td_label">
											<p>data52:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data52">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>data61:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data61">?</span></p>
										</td>
										<td class="td_label">
											<p>data62:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data62">?</span></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p>data71:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data71">?</span></p>
										</td>
										<td class="td_label">
											<p>data72:</p>
										</td>
										<td class="td_dynvar">
											<p><span id="data72">?</span></p>
										</td>
									</tr>
									-->
								</table>
							</fieldset>
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
							<fieldset>
								<legend>Paramètres</legend>
								<table class="table_sub">
									<tr>
										<td class="td_label">
											<p><label for="consi">consi:</label></p>
										</td>
										<td class="td_input">
											<p>
												<input type="radio" id="AWAY" name="consi" value="AWAY" />AWAY
												<br />
												<input type="radio" id="RAMP" name="consi" value="RAMP" />RAMP
											</p>
										</td>
										<td class="td_label">
											<p><label for="regu">regu:</label></p>
										</td>
										<td class="td_input">
											<p>
												<input type="radio" id="LR" name="regu" value="LR" />LR
												<br />
												<input type="radio" id="AT" name="regu" value="AT" />AT
											</p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p><label for="gotoX">gotoX:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="gotoX" name="gotoX" /></p>
										</td>
										<td class="td_label">
											<p><label for="gotoY">gotoY:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="gotoY" name="gotoY" /></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p><label for="adv">adv:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="adv" name="adv" /></p>
										</td>
										<td class="td_label">
											<p><label for="trn">trn:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="trn" name="trn" /></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p><label for="d1">d1:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="d1" name="d1" /></p>
										</td>
										<td class="td_label">
											<p><label for="d2">d2:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="d2" name="d2" /></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p><label for="vMax">vMax:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="vMax" name="vMax" /></p>
										</td>
										<td class="td_label">
											<p><label for="aMax">aMax:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="aMax" name="aMax" /></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p><label for="KP1">KP1:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="KP1" name="KP1" /></p>
										</td>
										<td class="td_label">
											<p><label for="KP2">KP2:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="KP2" name="KP2" /></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p><label for="KD1">KD1:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="KD1" name="KD1" /></p>
										</td>
										<td class="td_label">
											<p><label for="KD2">KD2:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="KD2" name="KD2" /></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p><label for="KI1">KI1:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="KI1" name="KI1" /></p>
										</td>
										<td class="td_label">
											<p><label for="KI2">KI2:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="KI2" name="KI2" /></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p><label for="ID">ID:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="ID" name="ID" /></p>
										</td>
										<td class="td_label">
											<p><label for="angle">angle:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" min="0" max="300" step="1" id="angle" name="angle" /></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p><label for="cor0">cor0:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="cor0" name="cor0" /></p>
										</td>
										<td class="td_label">
											<p><label for="cor1">cor1:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="cor1" name="cor1" /></p>
										</td>
									</tr>
									<tr>
										<td class="td_label">
											<p><label for="cor2">cor2:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="cor2" name="cor2" /></p>
										</td>
										<td class="td_label">
											<p><label for="cor3">cor3:</label></p>
										</td>
										<td class="td_input">
											<p><input class="input_number" type="number" step="0.1" id="cor3" name="cor3" /></p>
										</td>
									</tr>
								</table>
							</fieldset>
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
						</td>
						<td class="td_main_right">
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
							<!--
							<fieldset>
								<legend>Extra 2</legend>
								<table class="table_sub">
									<tr>
										<td>
											<div class="div_plot">
												<div class="div_placeholder" id="placeholder6"></div>
											</div>
										</td>
									</tr>
									<tr>
										<td>
											<div class="div_plot">
												<div class="div_placeholder" id="placeholder7"></div>
											</div>
										</td>
									</tr>
								</table>
							</fieldset>
							-->
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
							<fieldset>
								<legend>Carte</legend>
								<table class="table_sub">
									<tr>
										<td>
											<div class="div_plot">
												<div class="div_placeholder_map" id="placeholder3"></div>
											</div>
										</td>
									</tr>
								</table>
							</fieldset>
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
							<fieldset>
								<legend>Plot</legend>
								<table class="table_sub">
									<tr>
										<td>
											<div class="div_plot">
												<div class="div_placeholder" id="placeholder1"></div>
											</div>
										</td>
									</tr>
									<tr>
										<td>
											<div class="div_plot">
												<div class="div_placeholder" id="placeholder2"></div>
											</div>
										</td>
									</tr>
								</table>
							</fieldset>
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
							
							<fieldset>
								<legend>Log</legend>
								<table class="table_sub">
									<tr>
										<td class="td_log">
											<div class="div_log" id="dataTime1Log">dataTime1Log</div>
										</td>
										<td class="td_log">
											<div class="div_log" id="data11Log">data11Log</div>
										</td>
										<td class="td_log">
											<div class="div_log" id="data12Log">data12Log</div>
										</td>
									</tr>
									<tr>
										<td class="td_log">
											<div class="div_log" id="dataTime2Log">dataTime2Log</div>
										</td>
										<td class="td_log">
											<div class="div_log" id="data21Log">data21Log</div>
										</td>
										<td class="td_log">
											<div class="div_log" id="data22Log">data22Log</div>
										</td>
									</tr>
								</table>
							</fieldset>
							
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
							<!--
							<fieldset>
								<legend>Mode</legend>
								<div class="div_link">
									<h1><a href="demo.htm">DEMO</a></h1>
								</div>
							</fieldset>
							-->
							<!-- WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW -->
						</td>
					</tr>
					<tr>
						<td colspan="2">
							<div class="div_footer">
								<p>&copy2012 G4 Kraken</p>
							</div>
						</td>
					</tr>
				</table>
			</div>
		</div>
		</form>
	</center>
	<script type="text/javascript">
		// WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
		// Parses the xmlResponse from status.xml and updates the status box
		function updateStatus(xmlData) {
			var mainstat = document.getElementById('display').style.display;
			var loadstat = document.getElementById('loading').style.display;
			// Check if a timeout occurred
			if(!xmlData) {
				mainstat = 'none';
				loadstat = 'inline';
				return;
			}
			// Make sure we're displaying the status display
			mainstat = 'inline';
			loadstat = 'none';
			// Update the values
			document.getElementById('comL').innerHTML = getXMLValue(xmlData, 'comL');
			document.getElementById('comR').innerHTML = getXMLValue(xmlData, 'comR');
			document.getElementById('absoluteTime').innerHTML = getXMLValue(xmlData, 'absoluteTime');
			document.getElementById('localTime').innerHTML = getXMLValue(xmlData, 'localTime');
			document.getElementById('matchTime').innerHTML = getXMLValue(xmlData, 'matchTime');
			document.getElementById('time0').innerHTML = getXMLValue(xmlData, 'time0');
			document.getElementById('X0').innerHTML = getXMLValue(xmlData, 'X0');
			document.getElementById('Y0').innerHTML = getXMLValue(xmlData, 'Y0');
			document.getElementById('dir').innerHTML = getXMLValue(xmlData, 'dir');
			document.getElementById('led').innerHTML = getXMLValue(xmlData, 'led');
			document.getElementById('X2').innerHTML = getXMLValue(xmlData, 'X2');
			document.getElementById('Y2').innerHTML = getXMLValue(xmlData, 'Y2');
			document.getElementById('X1').innerHTML = getXMLValue(xmlData, 'X1');
			document.getElementById('Y1').innerHTML = getXMLValue(xmlData, 'Y1');
			document.getElementById('sharpFL').innerHTML = getXMLValue(xmlData, 'sharpFL');
			document.getElementById('sharpFR').innerHTML = getXMLValue(xmlData, 'sharpFR');
			document.getElementById('sharpBL').innerHTML = getXMLValue(xmlData, 'sharpBL');
			document.getElementById('sharpBR').innerHTML = getXMLValue(xmlData, 'sharpBR');
			document.getElementById('zoneL0').innerHTML = getXMLValue(xmlData, 'zoneL0');
			document.getElementById('zoneL1').innerHTML = getXMLValue(xmlData, 'zoneL1');
			document.getElementById('zoneL2').innerHTML = getXMLValue(xmlData, 'zoneL2');
			document.getElementById('zoneL3').innerHTML = getXMLValue(xmlData, 'zoneL3');
			document.getElementById('zoneL4').innerHTML = getXMLValue(xmlData, 'zoneL4');
			document.getElementById('zoneL5').innerHTML = getXMLValue(xmlData, 'zoneL5');
			document.getElementById('zoneR0').innerHTML = getXMLValue(xmlData, 'zoneR0');
			document.getElementById('zoneR1').innerHTML = getXMLValue(xmlData, 'zoneR1');
			document.getElementById('zoneR2').innerHTML = getXMLValue(xmlData, 'zoneR2');
			document.getElementById('zoneR3').innerHTML = getXMLValue(xmlData, 'zoneR3');
			document.getElementById('zoneR4').innerHTML = getXMLValue(xmlData, 'zoneR4');
			document.getElementById('zoneR5').innerHTML = getXMLValue(xmlData, 'zoneR5');
			document.getElementById('zoneM0').innerHTML = getXMLValue(xmlData, 'zoneM0');
			document.getElementById('zoneM1').innerHTML = getXMLValue(xmlData, 'zoneM1');
			document.getElementById('dataTime1').innerHTML = getXMLValue(xmlData, 'dataTime1');
			document.getElementById('data11').innerHTML = getXMLValue(xmlData, 'data11');
			document.getElementById('data12').innerHTML = getXMLValue(xmlData, 'data12');
			document.getElementById('dataTime2').innerHTML = getXMLValue(xmlData, 'dataTime2');
			document.getElementById('data21').innerHTML = getXMLValue(xmlData, 'data21');
			document.getElementById('data22').innerHTML = getXMLValue(xmlData, 'data22');
			/*
			document.getElementById('data41').innerHTML = getXMLValue(xmlData, 'data41');
			document.getElementById('data42').innerHTML = getXMLValue(xmlData, 'data42');
			document.getElementById('data51').innerHTML = getXMLValue(xmlData, 'data51');
			document.getElementById('data52').innerHTML = getXMLValue(xmlData, 'data52');
			document.getElementById('data61').innerHTML = getXMLValue(xmlData, 'data61');
			document.getElementById('data62').innerHTML = getXMLValue(xmlData, 'data62');
			document.getElementById('data71').innerHTML = getXMLValue(xmlData, 'data71');
			document.getElementById('data72').innerHTML = getXMLValue(xmlData, 'data72');
			*/
			document.getElementById('debug0').innerHTML = getXMLValue(xmlData, 'debug0');
			document.getElementById('debug1').innerHTML = getXMLValue(xmlData, 'debug1');
			document.getElementById('debug2').innerHTML = getXMLValue(xmlData, 'debug2');
			document.getElementById('debug3').innerHTML = getXMLValue(xmlData, 'debug3');
		}
		setTimeout("newAJAXCommand('status.xml', updateStatus, true)", 250);
		// WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
		var t;
		var i = 0;
		var en = 0;
		var color1 = "rgba(255, 255, 255, 0.6)";
		var color2 = "rgba(0, 128, 0, 0.6)";
		var color3 = "rgba(255, 165, 0, 0.6)";
		var color4 = "rgba(128, 0, 0, 0.6)";
		var color5 = "rgba(0, 0, 0, 0.6)";
		var zoneR0 = {label: "zoneR0", data: [[2500, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color1}, color: color1};
		var zoneR1 = {label: "zoneR1", data: [[2600, 1300, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color1}, color: color1};
		var zoneR2 = {label: "zoneR2", data: [[1700, 1000, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color1}, color: color1};
		var zoneR3 = {label: "zoneR3", data: [[1700, 1500, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color1}, color: color1};
		var zoneR4 = {label: "zoneR4", data: [[1700, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color1}, color: color1};
		var zoneR5 = {label: "zoneR5", data: [[2100, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color1}, color: color1};
		var zoneL0 = {label: "zoneL0", data: [[0, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color1}, color: color1};
		var zoneL1 = {label: "zoneL1", data: [[0, 1300, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color1}, color: color1};
		var zoneL2 = {label: "zoneL2", data: [[900, 1000, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color1}, color: color1};
		var zoneL3 = {label: "zoneL3", data: [[900, 1500, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color1}, color: color1};
		var zoneL4 = {label: "zoneL4", data: [[900, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color1}, color: color1};
		var zoneL5 = {label: "zoneL5", data: [[400, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color1}, color: color1};
		var zoneM0 = {label: "zoneM0", data: [[1100, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 800, fillColor: color1}, color: color1};
		var zoneM1 = {label: "zoneM1", data: [[1300, 2000, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color1}, color: color1};
		var series11 = {label: "data11", data: []};
		var series12 = {label: "data12", data: []};
		var series21 = {label: "data21", data: []};
		var series22 = {label: "data22", data: []};
		var series31 = {label: "data31", data: []};
		var series32 = {label: "data32", data: []};
		var series33 = {label: "data33", data: []};
		/*
		var series41 = {label: "data41", data: []};
		var series42 = {label: "data42", data: []};
		var series51 = {label: "data51", data: []};
		var series52 = {label: "data52", data: []};
		var series61 = {label: "data61", data: []};
		var series62 = {label: "data62", data: []};
		var series71 = {label: "data71", data: []};
		var series72 = {label: "data72", data: []};
		*/
		var data1 = [series11, series12];
		var data2 = [series21, series22];
		var data3 = [series31, series32, series33, zoneR0, zoneR1, zoneR2, zoneR3, zoneR4, zoneR5, zoneL0, zoneL1, zoneL2, zoneL3, zoneL4, zoneL5, zoneM0, zoneM1];
	    /*
		var data4 = [series41, series42];
		var data5 = [series51, series52];
		var data6 = [series61, series62];
		var data7 = [series71, series72];
		*/
		var options = {
			xaxis: {show: true, zoomRange: [0.001, 1000000], panRange: [-1000000, 1000000]},
			yaxis: {show: true, zoomRange: [0.001, 1000000], panRange: [-1000000, 1000000]},
			lines: {show: true},
			points: {show: false},
			legend: {show: true, position: "nw"},
			grid: {hoverable: true, clickable: true},
			series: {shadowSize: 0},
			zoom: {interactive: false},
			pan: {interactive: false}
		};
		var options2 = {		
			xaxis: {show: true, min: 0, max: 3000},
			yaxis: {show: true, min: 0, max: 2000},
			lines: {show: false},
			points: {show: true},
			legend: {show: false, position: "nw"},
			grid: {hoverable: true, clickable: true},
			series: {shadowSize: 0},
			zoom: {interactive: false},
			pan: {interactive: false}
		};
		var placeholder1 = $.plot($("#placeholder1"), data1, options);
		var placeholder2 = $.plot($("#placeholder2"), data2, options);
		var placeholder3 = $.plot($("#placeholder3"), data3, options2);
		/*
		var placeholder4 = $.plot($("#placeholder4"), data4, options);
		var placeholder5 = $.plot($("#placeholder5"), data5, options);
		var placeholder6 = $.plot($("#placeholder6"), data6, options);
		var placeholder7 = $.plot($("#placeholder7"), data7, options);
		*/
		function showTooltip(x, y, contents) {
			$('<div id="tooltip">' + contents + '</div>').css({
				position: 'absolute',
				display: 'none',
				top: y + 5,
				left: x + 5,
				border: '1px solid #666666',
				padding: '2px',
				'background-color': '#111111',
			}).appendTo("body").fadeIn(200);
		}
		var previousPoint = null;
		$("#placeholder1").bind("plothover", function (event, pos, item) {
			$("#x").text(pos.x.toFixed(2));
			$("#y").text(pos.y.toFixed(2));
            if(item) {
				if(previousPoint != item.dataIndex) {
					previousPoint = item.dataIndex;
					$("#tooltip").remove();
                    var x = item.datapoint[0].toFixed(2);
					var y = item.datapoint[1].toFixed(2);
                    showTooltip(item.pageX, item.pageY, item.series.label + ": (t=" + x + "; y=" + y + ")");
                }
            }
            else {
                $("#tooltip").remove();
                previousPoint = null;            
            }
		});
		$("#placeholder2").bind("plothover", function (event, pos, item) {
			$("#x").text(pos.x.toFixed(2));
			$("#y").text(pos.y.toFixed(2));
            if(item) {
				if(previousPoint != item.dataIndex) {
					previousPoint = item.dataIndex;
					$("#tooltip").remove();
                    var x = item.datapoint[0].toFixed(2);
					var y = item.datapoint[1].toFixed(2);
                    showTooltip(item.pageX, item.pageY, item.series.label + ": (t=" + x + "; y=" + y + ")");
                }
            }
            else {
                $("#tooltip").remove();
                previousPoint = null;            
            }
		});
		$("#placeholder3").bind("plothover", function (event, pos, item) {
			$("#x").text(pos.x.toFixed(2));
			$("#y").text(pos.y.toFixed(2));
            if(item) {
				if(previousPoint != item.dataIndex) {
					previousPoint = item.dataIndex;
					$("#tooltip").remove();
                    var x = item.datapoint[0].toFixed(2);
					var y = item.datapoint[1].toFixed(2);
                    showTooltip(item.pageX, item.pageY, item.series.label + ": (x=" + x + "; y=" + y + ")");
                }
            }
            else {
                $("#tooltip").remove();
                previousPoint = null;            
            }
		});
		/*
		$("#placeholder4").bind("plothover", function (event, pos, item) {
			$("#x").text(pos.x.toFixed(2));
			$("#y").text(pos.y.toFixed(2));
            if(item) {
				if(previousPoint != item.dataIndex) {
					previousPoint = item.dataIndex;
					$("#tooltip").remove();
                    var x = item.datapoint[0].toFixed(2);
					var y = item.datapoint[1].toFixed(2);
                    showTooltip(item.pageX, item.pageY, item.series.label + ": (t=" + x + "; y=" + y + ")");
                }
            }
            else {
                $("#tooltip").remove();
                previousPoint = null;            
            }
		});
		$("#placeholder5").bind("plothover", function (event, pos, item) {
			$("#x").text(pos.x.toFixed(2));
			$("#y").text(pos.y.toFixed(2));
            if(item) {
				if(previousPoint != item.dataIndex) {
					previousPoint = item.dataIndex;
					$("#tooltip").remove();
                    var x = item.datapoint[0].toFixed(2);
					var y = item.datapoint[1].toFixed(2);
                    showTooltip(item.pageX, item.pageY, item.series.label + ": (t=" + x + "; y=" + y + ")");
                }
            }
            else {
                $("#tooltip").remove();
                previousPoint = null;            
            }
		});
		$("#placeholder6").bind("plothover", function (event, pos, item) {
			$("#x").text(pos.x.toFixed(2));
			$("#y").text(pos.y.toFixed(2));
            if(item) {
				if(previousPoint != item.dataIndex) {
					previousPoint = item.dataIndex;
					$("#tooltip").remove();
                    var x = item.datapoint[0].toFixed(2);
					var y = item.datapoint[1].toFixed(2);
                    showTooltip(item.pageX, item.pageY, item.series.label + ": (t=" + x + "; y=" + y + ")");
                }
            }
            else {
                $("#tooltip").remove();
                previousPoint = null;            
            }
		});
		$("#placeholder7").bind("plothover", function (event, pos, item) {
			$("#x").text(pos.x.toFixed(2));
			$("#y").text(pos.y.toFixed(2));
            if(item) {
				if(previousPoint != item.dataIndex) {
					previousPoint = item.dataIndex;
					$("#tooltip").remove();
                    var x = item.datapoint[0].toFixed(2);
					var y = item.datapoint[1].toFixed(2);
                    showTooltip(item.pageX, item.pageY, item.series.label + ": (t=" + x + "; y=" + y + ")");
                }
            }
            else {
                $("#tooltip").remove();
                previousPoint = null;            
            }
		});
		*/
		function displayData() {
			// Log
			
			document.getElementById('dataTime1Log').innerHTML = document.getElementById('dataTime1Log').innerHTML
															+ "<br />"
															+ document.getElementById('dataTime1').innerHTML;
			document.getElementById('data11Log').innerHTML = document.getElementById('data11Log').innerHTML
															+ "<br />"
															+ document.getElementById('data11').innerHTML;
			document.getElementById('data12Log').innerHTML = document.getElementById('data12Log').innerHTML
															+ "<br />"
															+ document.getElementById('data12').innerHTML;
			document.getElementById('dataTime2Log').innerHTML = document.getElementById('dataTime2Log').innerHTML
															+ "<br />"
															+ document.getElementById('dataTime2').innerHTML;
			document.getElementById('data21Log').innerHTML = document.getElementById('data21Log').innerHTML
															+ "<br />"
															+ document.getElementById('data21').innerHTML;
			document.getElementById('data22Log').innerHTML = document.getElementById('data22Log').innerHTML
															+ "<br />"
															+ document.getElementById('data22').innerHTML;
			
			// Plot
			var xVal1 = document.getElementById('dataTime1').innerHTML;
			var xVal2 = document.getElementById('dataTime2').innerHTML;
			var xVal31 = document.getElementById('X0').innerHTML;
			var xVal32 = document.getElementById('X1').innerHTML;
			var xVal33 = document.getElementById('X2').innerHTML;
			var yVal11 = document.getElementById('data11').innerHTML;
			var yVal12 = document.getElementById('data12').innerHTML;
			var yVal21 = document.getElementById('data21').innerHTML;
			var yVal22 = document.getElementById('data22').innerHTML;
			var yVal31 = document.getElementById('Y0').innerHTML;
			var yVal32 = document.getElementById('Y1').innerHTML;
			var yVal33 = document.getElementById('Y2').innerHTML;
			/*
			var yVal41 = document.getElementById('data41').innerHTML;
			var yVal42 = document.getElementById('data42').innerHTML;
			var yVal51 = document.getElementById('data51').innerHTML;
			var yVal52 = document.getElementById('data52').innerHTML;
			var yVal61 = document.getElementById('data61').innerHTML;
			var yVal62 = document.getElementById('data62').innerHTML;
			var yVal71 = document.getElementById('data71').innerHTML;
			var yVal72 = document.getElementById('data72').innerHTML;
			*/
			var weightR0 = document.getElementById('zoneR0').innerHTML;
			var weightR1 = document.getElementById('zoneR1').innerHTML;
			var weightR2 = document.getElementById('zoneR2').innerHTML;
			var weightR3 = document.getElementById('zoneR3').innerHTML;
			var weightR4 = document.getElementById('zoneR4').innerHTML;
			var weightR5 = document.getElementById('zoneR5').innerHTML;
			var weightL0 = document.getElementById('zoneL0').innerHTML;
			var weightL1 = document.getElementById('zoneL1').innerHTML;
			var weightL2 = document.getElementById('zoneL2').innerHTML;
			var weightL3 = document.getElementById('zoneL3').innerHTML;
			var weightL4 = document.getElementById('zoneL4').innerHTML;
			var weightL5 = document.getElementById('zoneL5').innerHTML;
			var weightM0 = document.getElementById('zoneM0').innerHTML;
			var weightM1 = document.getElementById('zoneM1').innerHTML;
			
			if (weightR0 == 2) {data3[3] = {label: "zoneR0", data: [[2500, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color2}, color: color2};}
			if (weightR0 == 3) {data3[3] = {label: "zoneR0", data: [[2500, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color3}, color: color3};}
			if (weightR0 == 4) {data3[3] = {label: "zoneR0", data: [[2500, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color4}, color: color4};}
			if (weightR0 > 4) {data3[3] = {label: "zoneR0", data: [[2500, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color5}, color: color5};}

			if (weightR1 == 2) {data3[4] = {label: "zoneR1", data: [[2600, 1300, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color2}, color: color2};}
			if (weightR1 == 3) {data3[4] = {label: "zoneR1", data: [[2600, 1300, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color3}, color: color3};}
			if (weightR1 == 4) {data3[4] = {label: "zoneR1", data: [[2600, 1300, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color4}, color: color4};}
			if (weightR1 > 4) {data3[4] = {label: "zoneR1", data: [[2600, 1300, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color5}, color: color5};}
			
			if (weightR2 == 2) {data3[5] = {label: "zoneR2", data: [[1700, 1000, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color2}, color: color2};}
			if (weightR2 == 3) {data3[5] = {label: "zoneR2", data: [[1700, 1000, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color3}, color: color3};}
			if (weightR2 == 4) {data3[5] = {label: "zoneR2", data: [[1700, 1000, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color4}, color: color4};}
			if (weightR2 > 4) {data3[5] = {label: "zoneR2", data: [[1700, 1000, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color5}, color: color5};}

			if (weightR3 == 2) {data3[6] = {label: "zoneR3", data: [[1700, 1500, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color2}, color: color2};}
			if (weightR3 == 3) {data3[6] = {label: "zoneR3", data: [[1700, 1500, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color3}, color: color3};}
			if (weightR3 == 4) {data3[6] = {label: "zoneR3", data: [[1700, 1500, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color4}, color: color4};}
			if (weightR3 > 4) {data3[6] = {label: "zoneR3", data: [[1700, 1500, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color5}, color: color5};}

			if (weightR4 == 2) {data3[7] = {label: "zoneR4", data: [[1700, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color2}, color: color2};}
			if (weightR4 == 3) {data3[7] = {label: "zoneR4", data: [[1700, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color3}, color: color3};}
			if (weightR4 == 4) {data3[7] = {label: "zoneR4", data: [[1700, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color4}, color: color4};}
			if (weightR4 > 4) {data3[7] = {label: "zoneR4", data: [[1700, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color5}, color: color5};}

			if (weightR5 == 2) {data3[8] = {label: "zoneR5", data: [[2100, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color2}, color: color2};}
			if (weightR5 == 3) {data3[8] = {label: "zoneR5", data: [[2100, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color3}, color: color3};}
			if (weightR5 == 4) {data3[8] = {label: "zoneR5", data: [[2100, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color4}, color: color4};}
			if (weightR5 > 4) {data3[8] = {label: "zoneR5", data: [[2100, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color5}, color: color5};}

			if (weightL0 == 2) {data3[9] = {label: "zoneL0", data: [[0, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color2}, color: color2};}
			if (weightL0 == 3) {data3[9] = {label: "zoneL0", data: [[0, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color3}, color: color3};}
			if (weightL0 == 4) {data3[9] = {label: "zoneL0", data: [[0, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color4}, color: color4};}
			if (weightL0 > 4) {data3[9] = {label: "zoneL0", data: [[0, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color5}, color: color5};}
			
			if (weightL1 == 2) {data3[10] = {label: "zoneL1", data: [[0, 1300, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color2}, color: color2};}
			if (weightL1 == 3) {data3[10] = {label: "zoneL1", data: [[0, 1300, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color3}, color: color3};}
			if (weightL1 == 4) {data3[10] = {label: "zoneL1", data: [[0, 1300, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color4}, color: color4};}
			if (weightL1 > 4) {data3[10] = {label: "zoneL1", data: [[0, 1300, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color5}, color: color5};}
			
			if (weightL2 == 2) {data3[11] = {label: "zoneL2", data: [[900, 1000, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color2}, color: color2};}
			if (weightL2 == 3) {data3[11] = {label: "zoneL2", data: [[900, 1000, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color3}, color: color3};}
			if (weightL2 == 4) {data3[11] = {label: "zoneL2", data: [[900, 1000, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color4}, color: color4};}
			if (weightL2 > 4) {data3[11] = {label: "zoneL2", data: [[900, 1000, 500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color5}, color: color5};}
			
			if (weightL3 == 2) {data3[12] = {label: "zoneL3", data: [[900, 1500, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color2}, color: color2};}
			if (weightL3 == 3) {data3[12] = {label: "zoneL3", data: [[900, 1500, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color3}, color: color3};}
			if (weightL3 == 4) {data3[12] = {label: "zoneL3", data: [[900, 1500, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color4}, color: color4};}
			if (weightL3 > 4) {data3[12] = {label: "zoneL3", data: [[900, 1500, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color5}, color: color5};}

			if (weightL4 == 2) {data3[13] = {label: "zoneL4", data: [[900, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color2}, color: color2};}
			if (weightL4 == 3) {data3[13] = {label: "zoneL4", data: [[900, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color3}, color: color3};}
			if (weightL4 == 4) {data3[13] = {label: "zoneL4", data: [[900, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color4}, color: color4};}
			if (weightL4 > 4) {data3[13] = {label: "zoneL4", data: [[900, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color5}, color: color5};}

			if (weightL5 == 2) {data3[14] = {label: "zoneL5", data: [[400, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color2}, color: color2};}
			if (weightL5 == 3) {data3[14] = {label: "zoneL5", data: [[400, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color3}, color: color3};}
			if (weightL5 == 4) {data3[14] = {label: "zoneL5", data: [[400, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color4}, color: color4};}
			if (weightL5 > 4) {data3[14] = {label: "zoneL5", data: [[400, 2000, 1500]], points: {show: false}, bars: {show: true, barWidth: 500, fillColor: color5}, color: color5};}
			
			if (weightM0 == 2) {data3[15] = {label: "zoneM0", data: [[1100, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 800, fillColor: color2}, color: color2};}
			if (weightM0 == 3) {data3[15] = {label: "zoneM0", data: [[1100, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 800, fillColor: color3}, color: color3};}
			if (weightM0 == 4) {data3[15] = {label: "zoneM0", data: [[1100, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 800, fillColor: color4}, color: color4};}
			if (weightM0 > 4) {data3[15] = {label: "zoneM0", data: [[1100, 500, 0]], points: {show: false}, bars: {show: true, barWidth: 800, fillColor: color5}, color: color5};}

			if (weightM1 == 2) {data3[16] = {label: "zoneM1", data: [[1300, 2000, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color2}, color: color2};}
			if (weightM1 == 3) {data3[16] = {label: "zoneM1", data: [[1300, 2000, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color3}, color: color3};}
			if (weightM1 == 4) {data3[16] = {label: "zoneM1", data: [[1300, 2000, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color4}, color: color4};}
			if (weightM1 > 4) {data3[16] = {label: "zoneM1", data: [[1300, 2000, 1000]], points: {show: false}, bars: {show: true, barWidth: 400, fillColor: color5}, color: color5};}

			var datum11 = [xVal1, yVal11];
			var datum12 = [xVal1, yVal12];
			var datum21 = [xVal2, yVal21];
			var datum22 = [xVal2, yVal22];
			var datum31 = [xVal31, yVal31];
			var datum32 = [xVal32, yVal32];
			var datum33 = [xVal33, yVal33];
			/*
			var datum41 = [xVal1, yVal41];
			var datum42 = [xVal1, yVal42];
			var datum51 = [xVal1, yVal51];
			var datum52 = [xVal1, yVal52];
			var datum61 = [xVal1, yVal61];
			var datum62 = [xVal1, yVal62];
			var datum71 = [xVal1, yVal71];
			var datum72 = [xVal1, yVal72];
			*/
			data1[0].data.push(datum11);
			data1[1].data.push(datum12);
			data2[0].data.push(datum21);
			data2[1].data.push(datum22);
			data3[0].data.push(datum31);
			data3[1].data.push(datum32);
			data3[2].data.push(datum33);
			/*
			data4[0].data.push(datum41);
			data4[1].data.push(datum42);
			data5[0].data.push(datum51);
			data5[1].data.push(datum52);
			data6[0].data.push(datum61);
			data6[1].data.push(datum62);
			data7[0].data.push(datum71);
			data7[1].data.push(datum72);
			*/
			if(data1[0].data.length > 100) {
				// only allow 150 points
				data1[0].data = data1[0].data.splice(1);
				data1[1].data = data1[1].data.splice(1);
			}
			if(data2[0].data.length > 100) {
				// only allow 150 points
				data2[0].data = data2[0].data.splice(1);
				data2[1].data = data2[1].data.splice(1);
			}
			if(data3[0].data.length > 1) {
				// only allow 5 points
				data3[0].data = data3[0].data.splice(1);
				data3[1].data = data3[1].data.splice(1);
				data3[2].data = data3[2].data.splice(1);
			}
			/*
			if(data4[0].data.length > 100) {
				// only allow 150 points
				data4[0].data = data4[0].data.splice(1);
				data4[1].data = data4[1].data.splice(1);
			}
			if(data5[0].data.length > 100) {
				// only allow 150 points
				data5[0].data = data5[0].data.splice(1);
				data5[1].data = data5[1].data.splice(1);
			}
			if(data6[0].data.length > 100) {
				// only allow 150 points
				data6[0].data = data6[0].data.splice(1);
				data6[1].data = data6[1].data.splice(1);
			}
			if(data7[0].data.length > 100) {
				// only allow 150 points
				data7[0].data = data7[0].data.splice(1);
				data7[1].data = data7[1].data.splice(1);
			}
			*/
			placeholder1.setData(data1);
			placeholder1.setupGrid();
			placeholder1.draw();
			placeholder2.setData(data2);
			placeholder2.setupGrid();
			placeholder2.draw();
			placeholder3.setData(data3);
			placeholder3.setupGrid();
			placeholder3.draw();
			/*
			placeholder4.setData(data4);
			placeholder4.setupGrid();
			placeholder4.draw();
			placeholder5.setData(data5);
			placeholder5.setupGrid();
			placeholder5.draw();
			placeholder6.setData(data6);
			placeholder6.setupGrid();
			placeholder6.draw();
			placeholder7.setData(data7);
			placeholder7.setupGrid();
			placeholder7.draw();
			*/
			i = i + 250;
			if(i < 60000) {
				// Refresh rate
				t = setTimeout("displayData()", 250);
			}
		}
		// WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
		function startData() {
			if(!en) {
				en=1;
				displayData();
			}
		}
		function stopData() {
			clearTimeout(t);
			en=0;
		}
		// WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
		function displayLastInput() {
			document.getElementById('gotoX').value = gup('gotoX');
			document.getElementById('gotoY').value = gup('gotoY');
			document.getElementById('adv').value = gup('adv');
			document.getElementById('trn').value = gup('trn');
			document.getElementById('KP1').value = gup('KP1');
			document.getElementById('KP2').value = gup('KP2');
			document.getElementById('KD1').value = gup('KD1');
			document.getElementById('KD2').value = gup('KD2');
			document.getElementById('KI1').value = gup('KI1');
			document.getElementById('KI2').value = gup('KI2');
			document.getElementById('d1').value = gup('d1');
			document.getElementById('d2').value = gup('d2');
			document.getElementById('vMax').value = gup('vMax');
			document.getElementById('aMax').value = gup('aMax');
			document.getElementById('ID').value = gup('ID');
			document.getElementById('angle').value = gup('angle');
			document.getElementById('cor0').value = gup('cor0');
			document.getElementById('cor1').value = gup('cor1');
			document.getElementById('cor2').value = gup('cor2');
			document.getElementById('cor3').value = gup('cor3');
			if(gup('consi') == "AWAY") {document.getElementById('AWAY').checked = true;}
			if(gup('consi') == "RAMP") {document.getElementById('RAMP').checked = true;}
			if(gup('regu') == "LR") {document.getElementById('LR').checked = true;}
			if(gup('regu') == "AT") {document.getElementById('AT').checked = true;}
		}
		function gup(name) {
			name = name.replace(/[\[]/,"\\\[").replace(/[\]]/,"\\\]");
			var regexS = "[\\?&]"+name+"=([^&#]*)";
			var regex = new RegExp(regexS);
			var results = regex.exec(window.location.href);
			if(results == null) {
				return "";
			}
			else {
				return results[1];
			}
		}
		// WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
		window.onload = function() {
			displayLastInput();
			displayData();
		}
	</script>
</body>
</html>
"""

print(html)
