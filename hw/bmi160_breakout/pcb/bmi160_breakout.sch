<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="6.5.0">
<drawing>
<settings>
<setting alwaysvectorfont="yes"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="bmi160_breakout">
<packages>
<package name="LGA14">
<smd name="13" x="0" y="1.025" dx="0.675" dy="0.25" layer="1" rot="R90"/>
<smd name="12" x="0.5" y="1.025" dx="0.675" dy="0.25" layer="1" rot="R90"/>
<smd name="14" x="-0.5" y="1.025" dx="0.675" dy="0.25" layer="1" rot="R90"/>
<smd name="6" x="0" y="-1.025" dx="0.675" dy="0.25" layer="1" rot="R90"/>
<smd name="7" x="0.5" y="-1.025" dx="0.675" dy="0.25" layer="1" rot="R90"/>
<smd name="5" x="-0.5" y="-1.025" dx="0.675" dy="0.25" layer="1" rot="R90"/>
<smd name="2" x="-1.275" y="0.25" dx="0.675" dy="0.25" layer="1" rot="R180"/>
<smd name="3" x="-1.275" y="-0.25" dx="0.675" dy="0.25" layer="1" rot="R180"/>
<smd name="1" x="-1.275" y="0.75" dx="0.675" dy="0.25" layer="1" rot="R180"/>
<smd name="4" x="-1.275" y="-0.75" dx="0.675" dy="0.25" layer="1" rot="R180"/>
<smd name="11" x="1.275" y="0.75" dx="0.675" dy="0.25" layer="1" rot="R180"/>
<smd name="10" x="1.275" y="0.25" dx="0.675" dy="0.25" layer="1" rot="R180"/>
<smd name="9" x="1.275" y="-0.25" dx="0.675" dy="0.25" layer="1" rot="R180"/>
<smd name="8" x="1.275" y="-0.75" dx="0.675" dy="0.25" layer="1" rot="R180"/>
<wire x1="-1.5" y1="1.25" x2="-1.25" y2="1.25" width="0.127" layer="21"/>
<wire x1="-1.25" y1="1.25" x2="1.5" y2="1.25" width="0.127" layer="21"/>
<wire x1="1.5" y1="1.25" x2="1.5" y2="-1.25" width="0.127" layer="21"/>
<wire x1="1.5" y1="-1.25" x2="-1.5" y2="-1.25" width="0.127" layer="21"/>
<wire x1="-1.5" y1="-1.25" x2="-1.5" y2="1" width="0.127" layer="21"/>
<wire x1="-1.5" y1="1" x2="-1.5" y2="1.25" width="0.127" layer="21"/>
<wire x1="-1.5" y1="1" x2="-1.25" y2="1.25" width="0.127" layer="21"/>
<text x="-1.4" y="1.6" size="0.8" layer="21">&gt;NAME</text>
<circle x="-0.525" y="0.3" radius="0.111803125" width="0.127" layer="21"/>
</package>
<package name="B4B-ZR">
<description>&lt;b&gt;ZH CONNECTOR&lt;/b&gt;  Top entry type, 1.5 mm, 4 pin 1 row&lt;p&gt;
Source: http://www.jst.com .. eZH.pdf</description>
<wire x1="0" y1="0.07" x2="7.5" y2="0.07" width="0.127" layer="21"/>
<wire x1="7.5" y1="0.07" x2="7.5" y2="-3.43" width="0.127" layer="21"/>
<wire x1="7.5" y1="-3.43" x2="0" y2="-3.43" width="0.127" layer="21"/>
<wire x1="0" y1="-3.43" x2="0" y2="0.07" width="0.127" layer="21"/>
<text x="-0.75" y="0.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.75" y="-5.365" size="1.27" layer="27">&gt;VALUE</text>
<pad name="4" x="1.5" y="-1.3" drill="0.7"/>
<pad name="3" x="3" y="-1.3" drill="0.7"/>
<pad name="2" x="4.5" y="-1.3" drill="0.7"/>
<pad name="1" x="6" y="-1.3" drill="0.7"/>
</package>
</packages>
<symbols>
<symbol name="BMI160">
<wire x1="-12.7" y1="20.32" x2="-12.7" y2="-22.86" width="0.254" layer="94"/>
<wire x1="-12.7" y1="-22.86" x2="12.7" y2="-22.86" width="0.254" layer="94"/>
<wire x1="12.7" y1="-22.86" x2="12.7" y2="20.32" width="0.254" layer="94"/>
<wire x1="12.7" y1="20.32" x2="-12.7" y2="20.32" width="0.254" layer="94"/>
<text x="-10.16" y="15.24" size="1.778" layer="94">BMI160</text>
<text x="-10.16" y="22.86" size="1.778" layer="94">&gt;NAME</text>
<pin name="VDD" x="-17.78" y="5.08" length="middle"/>
<pin name="VDDIO" x="-17.78" y="0" length="middle"/>
<pin name="GND" x="-17.78" y="-10.16" length="middle"/>
<pin name="GNDIO" x="-17.78" y="-15.24" length="middle"/>
<pin name="OSCB" x="17.78" y="-17.78" length="middle" rot="R180"/>
<pin name="OSDO" x="17.78" y="-15.24" length="middle" rot="R180"/>
<pin name="CSB" x="17.78" y="-10.16" length="middle" rot="R180"/>
<pin name="ASCX" x="17.78" y="-5.08" length="middle" rot="R180"/>
<pin name="ASDX" x="17.78" y="-2.54" length="middle" rot="R180"/>
<pin name="SDO/SA0" x="17.78" y="10.16" length="middle" rot="R180"/>
<pin name="SCX/SCL" x="17.78" y="12.7" length="middle" rot="R180"/>
<pin name="SDX/SDA" x="17.78" y="15.24" length="middle" rot="R180"/>
<pin name="INT2" x="17.78" y="5.08" length="middle" rot="R180"/>
<pin name="INT1" x="17.78" y="2.54" length="middle" rot="R180"/>
</symbol>
<symbol name="B4B-ZR">
<pin name="1" x="-7.62" y="2.54" visible="pin" length="middle"/>
<pin name="2" x="-7.62" y="0" visible="pin" length="middle"/>
<pin name="3" x="-7.62" y="-2.54" visible="pin" length="middle"/>
<pin name="4" x="-7.62" y="-5.08" visible="pin" length="middle"/>
<text x="-2.54" y="5.842" size="1.27" layer="95">&gt;NAME</text>
<text x="-2.54" y="-9.398" size="1.27" layer="96">&gt;VALUE</text>
<wire x1="-5.08" y1="5.08" x2="2.54" y2="5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="5.08" x2="2.54" y2="-7.62" width="0.254" layer="94"/>
<wire x1="2.54" y1="-7.62" x2="-5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="-5.08" y1="-7.62" x2="-5.08" y2="5.08" width="0.254" layer="94"/>
<circle x="-2.54" y="2.54" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="0" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="-2.54" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="-5.08" radius="0.915809375" width="0.254" layer="94"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="BMI160">
<gates>
<gate name="G$1" symbol="BMI160" x="0" y="0"/>
</gates>
<devices>
<device name="LGA14" package="LGA14">
<connects>
<connect gate="G$1" pin="ASCX" pad="3"/>
<connect gate="G$1" pin="ASDX" pad="2"/>
<connect gate="G$1" pin="CSB" pad="12"/>
<connect gate="G$1" pin="GND" pad="7"/>
<connect gate="G$1" pin="GNDIO" pad="6"/>
<connect gate="G$1" pin="INT1" pad="4"/>
<connect gate="G$1" pin="INT2" pad="9"/>
<connect gate="G$1" pin="OSCB" pad="10"/>
<connect gate="G$1" pin="OSDO" pad="11"/>
<connect gate="G$1" pin="SCX/SCL" pad="13"/>
<connect gate="G$1" pin="SDO/SA0" pad="1"/>
<connect gate="G$1" pin="SDX/SDA" pad="14"/>
<connect gate="G$1" pin="VDD" pad="8"/>
<connect gate="G$1" pin="VDDIO" pad="5"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="B4B-ZR" prefix="CON">
<description>&lt;b&gt;JST-ZH Connector&lt;/b&gt;&lt;br&gt;
4 pol, 1.5mm pitch&lt;br&gt;
Package: &lt;b&gt;B4B-ZR&lt;/b&gt;
&lt;br&gt;&lt;br&gt;
Source: &lt;a href="http://www.jst-mfg.com/product/pdf/eng/eZH.pdf"&gt;http://www.jst-mfg.com/product/pdf/eng/eZH.pdf&lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="B4B-ZR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="B4B-ZR">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply1">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
 GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
 Please keep in mind, that these devices are necessary for the
 automatic wiring of the supply signals.&lt;p&gt;
 The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
 In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
 &lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GND">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="VDD">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<wire x1="0" y1="1.27" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<wire x1="1.27" y1="-1.905" x2="0" y2="1.27" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="VDD" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" prefix="GND">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="VDD" prefix="VDD">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="VDD" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="mfpassives">
<packages>
<package name="C0402">
<description>&lt;b&gt;Description:&lt;/b&gt; Standard 0402 Package for Capacitors&lt;br/&gt;</description>
<smd name="P$1" x="-0.55" y="0" dx="0.5" dy="0.6" layer="1" rot="R180"/>
<smd name="P$2" x="0.55" y="0" dx="0.5" dy="0.6" layer="1" rot="R180"/>
<wire x1="-1.1" y1="0.55" x2="-1.1" y2="-0.55" width="0.127" layer="21"/>
<wire x1="-1.1" y1="-0.55" x2="1.1" y2="-0.55" width="0.127" layer="21"/>
<wire x1="1.1" y1="-0.55" x2="1.1" y2="0.55" width="0.127" layer="21"/>
<wire x1="1.1" y1="0.55" x2="-1.1" y2="0.55" width="0.127" layer="21"/>
<text x="-1.1" y="1.1" size="0.8128" layer="25" font="vector" ratio="16">&gt;NAME</text>
</package>
<package name="C0603">
<description>&lt;b&gt;Description:&lt;/b&gt; Standard 0603 Package for Capacitors&lt;br/&gt;</description>
<smd name="P$1" x="-0.75" y="0" dx="0.6" dy="0.9" layer="1" rot="R180"/>
<smd name="P$2" x="0.75" y="0" dx="0.6" dy="0.9" layer="1" rot="R180"/>
<wire x1="-1.4" y1="0.7" x2="-1.4" y2="-0.7" width="0.127" layer="21"/>
<wire x1="-1.4" y1="-0.7" x2="1.4" y2="-0.7" width="0.127" layer="21"/>
<wire x1="1.4" y1="-0.7" x2="1.4" y2="0.7" width="0.127" layer="21"/>
<wire x1="1.4" y1="0.7" x2="-1.4" y2="0.7" width="0.127" layer="21"/>
<text x="-1.4" y="1.1" size="0.8" layer="25" font="vector" ratio="10">&gt;NAME</text>
</package>
<package name="C0805">
<description>&lt;b&gt;Description:&lt;/b&gt; Standard 0805 Package for Capacitors&lt;br/&gt;</description>
<smd name="P$1" x="-0.95" y="0" dx="0.7" dy="1.3" layer="1" rot="R180"/>
<smd name="P$2" x="0.95" y="0" dx="0.7" dy="1.3" layer="1" rot="R180"/>
<wire x1="-1.8" y1="0.9" x2="-1.8" y2="-0.9" width="0.127" layer="21"/>
<wire x1="-1.8" y1="-0.9" x2="1.8" y2="-0.9" width="0.127" layer="21"/>
<wire x1="1.8" y1="-0.9" x2="1.8" y2="0.9" width="0.127" layer="21"/>
<wire x1="1.8" y1="0.9" x2="-1.8" y2="0.9" width="0.127" layer="21"/>
<text x="-1.5" y="1.1" size="0.8128" layer="25" font="vector" ratio="16">&gt;NAME</text>
</package>
<package name="C1206">
<description>&lt;b&gt;Description:&lt;/b&gt; Standard 1206 Package for Capacitors&lt;br/&gt;</description>
<smd name="P$1" x="-1.45" y="0" dx="0.9" dy="1.6" layer="1" rot="R180"/>
<smd name="P$2" x="1.45" y="0" dx="0.9" dy="1.6" layer="1" rot="R180"/>
<wire x1="-2.2" y1="1.1" x2="-2.2" y2="-1.1" width="0.127" layer="21"/>
<wire x1="-2.2" y1="-1.1" x2="2.2" y2="-1.1" width="0.127" layer="21"/>
<wire x1="2.2" y1="-1.1" x2="2.2" y2="1.1" width="0.127" layer="21"/>
<wire x1="2.2" y1="1.1" x2="-2.2" y2="1.1" width="0.127" layer="21"/>
<text x="-2.2" y="1.3" size="0.8128" layer="25" font="vector" ratio="16">&gt;NAME</text>
</package>
<package name="C1210">
<description>&lt;b&gt;Description:&lt;/b&gt; Standard 1210 Package for Capacitors&lt;br/&gt;</description>
<smd name="P$1" x="-1.45" y="0" dx="0.9" dy="2.5" layer="1" rot="R180"/>
<smd name="P$2" x="1.45" y="0" dx="0.9" dy="2.5" layer="1" rot="R180"/>
<wire x1="-2.2" y1="1.6" x2="-2.2" y2="-1.575" width="0.127" layer="21"/>
<wire x1="-2.2" y1="-1.575" x2="2.2" y2="-1.575" width="0.127" layer="21"/>
<wire x1="2.2" y1="-1.575" x2="2.2" y2="1.6" width="0.127" layer="21"/>
<wire x1="2.2" y1="1.6" x2="-2.2" y2="1.6" width="0.127" layer="21"/>
<text x="-2.2" y="1.8" size="0.8128" layer="25" font="vector" ratio="16">&gt;NAME</text>
</package>
</packages>
<symbols>
<symbol name="CAPACITOR_NP">
<description>&lt;b&gt;Library:&lt;/b&gt;  MF_Passives&lt;br/&gt;
&lt;b&gt;Description:&lt;/b&gt; Symbol for Non-Polarized Capacitors&lt;br/&gt;</description>
<pin name="P$1" x="0" y="5.08" visible="off" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="P$2" x="0" y="-5.08" visible="off" length="short" direction="pas" swaplevel="1" rot="R90"/>
<text x="2.54" y="2.54" size="1.016" layer="95" font="vector">&gt;NAME</text>
<text x="2.54" y="-2.54" size="1.016" layer="96" font="vector" align="top-left">&gt;VALUE</text>
<wire x1="0" y1="2.54" x2="0" y2="1.016" width="0.1524" layer="94"/>
<wire x1="0" y1="-2.54" x2="0" y2="-1.016" width="0.1524" layer="94"/>
<rectangle x1="-1.778" y1="0.508" x2="1.778" y2="1.27" layer="94"/>
<rectangle x1="-1.778" y1="-1.27" x2="1.778" y2="-0.508" layer="94"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="CAPACITOR_NP" prefix="C" uservalue="yes">
<description>&lt;b&gt;Library:&lt;/b&gt;  MF_Passives&lt;br/&gt;
&lt;b&gt;Description:&lt;/b&gt; Device for Non-Polarized Capacitors. Manufacture part number (MFG#), Voltage, Temperature Coefficient, and Tolerance can be added via Attributes.&lt;br/&gt;</description>
<gates>
<gate name="G$1" symbol="CAPACITOR_NP" x="0" y="0"/>
</gates>
<devices>
<device name="_0402" package="C0402">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
</connects>
<technologies>
<technology name="">
<attribute name="MFG#" value="N/A" constant="no"/>
<attribute name="TEMP_COF" value="N/A" constant="no"/>
<attribute name="TOLERANCE" value="N/A" constant="no"/>
<attribute name="VOLTAGE" value="N/A" constant="no"/>
</technology>
</technologies>
</device>
<device name="_0603" package="C0603">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
</connects>
<technologies>
<technology name="">
<attribute name="MFG#" value="N/A" constant="no"/>
<attribute name="TEMP_COF" value="N/A" constant="no"/>
<attribute name="TOLERANCE" value="N/A" constant="no"/>
<attribute name="VOLTAGE" value="N/A" constant="no"/>
</technology>
</technologies>
</device>
<device name="_0805" package="C0805">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
</connects>
<technologies>
<technology name="">
<attribute name="MFG#" value="N/A" constant="no"/>
<attribute name="TEMP_COF" value="N/A" constant="no"/>
<attribute name="TOLERANCE" value="N/A" constant="no"/>
<attribute name="VOLTAGE" value="N/A" constant="no"/>
</technology>
</technologies>
</device>
<device name="_1206" package="C1206">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
</connects>
<technologies>
<technology name="">
<attribute name="MFG#" value="N/A" constant="no"/>
<attribute name="TEMP_COF" value="N/A" constant="no"/>
<attribute name="TOLERANCE" value="N/A" constant="no"/>
<attribute name="VOLTAGE" value="N/A" constant="no"/>
</technology>
</technologies>
</device>
<device name="_1210" package="C1210">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
</connects>
<technologies>
<technology name="">
<attribute name="MFG#" value="N/A" constant="no"/>
<attribute name="TEMP_COF" value="N/A" constant="no"/>
<attribute name="TOLERANCE" value="N/A" constant="no"/>
<attribute name="VOLTAGE" value="N/A" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="jumper">
<description>&lt;b&gt;Jumpers&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="SJ_2W">
<description>&lt;b&gt;Solder jumper&lt;/b&gt;</description>
<wire x1="3.175" y1="-1.524" x2="-3.175" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.524" x2="3.429" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.429" y1="1.27" x2="-3.175" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.429" y1="-1.27" x2="-3.175" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="3.175" y1="-1.524" x2="3.429" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="3.429" y1="-1.27" x2="3.429" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.27" x2="-3.429" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.524" x2="3.175" y2="1.524" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="-2.794" y1="0" x2="-3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="0" y1="0.762" x2="0" y2="1.397" width="0.1524" layer="51"/>
<wire x1="0" y1="-1.397" x2="0" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="2.032" y1="0.127" x2="2.032" y2="-0.127" width="1.27" layer="51" curve="-180" cap="flat"/>
<wire x1="-2.032" y1="-0.127" x2="-2.032" y2="0.127" width="1.27" layer="51" curve="-180" cap="flat"/>
<smd name="1" x="-2.54" y="0" dx="1.27" dy="2.54" layer="1"/>
<smd name="2" x="0" y="0" dx="1.27" dy="2.54" layer="1"/>
<smd name="3" x="2.54" y="0" dx="1.27" dy="2.54" layer="1"/>
<text x="-3.429" y="1.778" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.1001" y="0" size="0.02" layer="27">&gt;VALUE</text>
<rectangle x1="-0.508" y1="-0.762" x2="0.508" y2="0.762" layer="51"/>
</package>
<package name="SJ_2">
<description>&lt;b&gt;Solder jumper&lt;/b&gt;</description>
<wire x1="2.159" y1="-1.016" x2="-2.159" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.016" x2="2.413" y2="0.762" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="0.762" x2="-2.159" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="-0.762" x2="-2.159" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="2.159" y1="-1.016" x2="2.413" y2="-0.762" width="0.1524" layer="21" curve="90"/>
<wire x1="2.413" y1="-0.762" x2="2.413" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-0.762" x2="-2.413" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.016" x2="2.159" y2="1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0" x2="2.286" y2="0" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="0" x2="-2.286" y2="0" width="0.1524" layer="51"/>
<wire x1="0" y1="0.762" x2="0" y2="1.016" width="0.1524" layer="51"/>
<wire x1="0" y1="-1.016" x2="0" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="1.016" y1="0.127" x2="1.016" y2="-0.127" width="1.27" layer="51" curve="-180" cap="flat"/>
<wire x1="-1.016" y1="-0.127" x2="-1.016" y2="0.127" width="1.27" layer="51" curve="-180" cap="flat"/>
<smd name="1" x="-1.524" y="0" dx="1.1684" dy="1.6002" layer="1"/>
<smd name="2" x="0" y="0" dx="1.1684" dy="1.6002" layer="1"/>
<smd name="3" x="1.524" y="0" dx="1.1684" dy="1.6002" layer="1"/>
<text x="-2.413" y="1.27" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.1001" y="0" size="0.02" layer="27">&gt;VALUE</text>
<rectangle x1="-0.508" y1="-0.762" x2="0.508" y2="0.762" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="SJ_2">
<wire x1="-0.635" y1="-1.397" x2="0.635" y2="-1.397" width="1.27" layer="94" curve="180" cap="flat"/>
<wire x1="-0.635" y1="1.397" x2="0.635" y2="1.397" width="1.27" layer="94" curve="-180" cap="flat"/>
<wire x1="1.27" y1="-0.635" x2="-1.27" y2="-0.635" width="0.1524" layer="94"/>
<wire x1="-1.27" y1="-0.635" x2="-1.27" y2="0" width="0.1524" layer="94"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="0.635" width="0.1524" layer="94"/>
<wire x1="-1.27" y1="0.635" x2="1.27" y2="0.635" width="0.1524" layer="94"/>
<wire x1="1.27" y1="0.635" x2="1.27" y2="-0.635" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.1524" layer="94"/>
<text x="2.54" y="0.381" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-1.905" size="1.778" layer="96">&gt;VALUE</text>
<rectangle x1="-1.27" y1="-0.635" x2="1.27" y2="0.635" layer="94"/>
<pin name="3" x="0" y="-5.08" visible="pad" length="short" direction="pas" rot="R90"/>
<pin name="1" x="0" y="5.08" visible="pad" length="short" direction="pas" rot="R270"/>
<pin name="2" x="-5.08" y="0" visible="pad" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="SJ2W" prefix="SJ" uservalue="yes">
<description>SMD solder &lt;b&gt;JUMPER&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="SJ_2" x="0" y="0"/>
</gates>
<devices>
<device name="W" package="SJ_2W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="" package="SJ_2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0.2032" drill="0">
</class>
</classes>
<parts>
<part name="U1" library="bmi160_breakout" deviceset="BMI160" device="LGA14">
<attribute name="MOUSER_PN" value="262-BMI160"/>
<attribute name="MPN" value="BMI160"/>
</part>
<part name="GND1" library="supply1" deviceset="GND" device=""/>
<part name="VDD1" library="supply1" deviceset="VDD" device=""/>
<part name="C1" library="mfpassives" deviceset="CAPACITOR_NP" device="_0603" value=".1uF">
<attribute name="AMFPN" value="MF-CAP-0603-0.1uF"/>
</part>
<part name="C2" library="mfpassives" deviceset="CAPACITOR_NP" device="_0805" value="1uF">
<attribute name="AMFPN" value="MF-CAP-0805-1uF"/>
</part>
<part name="CON1" library="bmi160_breakout" deviceset="B4B-ZR" device=""/>
<part name="SJ1" library="jumper" deviceset="SJ2W" device=""/>
<part name="GND2" library="supply1" deviceset="GND" device=""/>
<part name="VDD2" library="supply1" deviceset="VDD" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U1" gate="G$1" x="68.58" y="43.18">
<attribute name="MOUSER_PN" x="68.58" y="43.18" size="1.778" layer="96" display="off"/>
<attribute name="MPN" x="68.58" y="43.18" size="1.778" layer="96" display="off"/>
</instance>
<instance part="GND1" gate="1" x="60.96" y="5.08"/>
<instance part="VDD1" gate="G$1" x="38.1" y="83.82"/>
<instance part="C1" gate="G$1" x="40.64" y="33.02">
<attribute name="AMFPN" x="40.64" y="33.02" size="1.778" layer="96" display="off"/>
</instance>
<instance part="C2" gate="G$1" x="27.94" y="33.02">
<attribute name="AMFPN" x="27.94" y="33.02" size="1.778" layer="96" display="off"/>
</instance>
<instance part="CON1" gate="G$1" x="139.7" y="43.18"/>
<instance part="SJ1" gate="G$1" x="129.54" y="66.04"/>
<instance part="GND2" gate="1" x="129.54" y="55.88"/>
<instance part="VDD2" gate="G$1" x="129.54" y="76.2"/>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="GNDIO"/>
<wire x1="50.8" y1="27.94" x2="48.26" y2="27.94" width="0.1524" layer="91"/>
<wire x1="48.26" y1="27.94" x2="48.26" y2="10.16" width="0.1524" layer="91"/>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="48.26" y1="10.16" x2="60.96" y2="10.16" width="0.1524" layer="91"/>
<wire x1="60.96" y1="10.16" x2="60.96" y2="7.62" width="0.1524" layer="91"/>
<pinref part="U1" gate="G$1" pin="GND"/>
<wire x1="50.8" y1="33.02" x2="48.26" y2="33.02" width="0.1524" layer="91"/>
<wire x1="48.26" y1="33.02" x2="48.26" y2="27.94" width="0.1524" layer="91"/>
<junction x="48.26" y="27.94"/>
<pinref part="C1" gate="G$1" pin="P$2"/>
<wire x1="40.64" y1="27.94" x2="40.64" y2="10.16" width="0.1524" layer="91"/>
<wire x1="40.64" y1="10.16" x2="48.26" y2="10.16" width="0.1524" layer="91"/>
<junction x="48.26" y="10.16"/>
<pinref part="C2" gate="G$1" pin="P$2"/>
<wire x1="27.94" y1="27.94" x2="27.94" y2="10.16" width="0.1524" layer="91"/>
<wire x1="27.94" y1="10.16" x2="40.64" y2="10.16" width="0.1524" layer="91"/>
<junction x="40.64" y="10.16"/>
<junction x="60.96" y="10.16"/>
<wire x1="60.96" y1="10.16" x2="129.54" y2="10.16" width="0.1524" layer="91"/>
<wire x1="129.54" y1="10.16" x2="129.54" y2="38.1" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="4"/>
<wire x1="129.54" y1="38.1" x2="132.08" y2="38.1" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SJ1" gate="G$1" pin="3"/>
<pinref part="GND2" gate="1" pin="GND"/>
<wire x1="129.54" y1="60.96" x2="129.54" y2="58.42" width="0.1524" layer="91"/>
</segment>
</net>
<net name="VDD" class="0">
<segment>
<pinref part="C1" gate="G$1" pin="P$1"/>
<wire x1="40.64" y1="38.1" x2="40.64" y2="43.18" width="0.1524" layer="91"/>
<pinref part="U1" gate="G$1" pin="VDDIO"/>
<wire x1="40.64" y1="43.18" x2="50.8" y2="43.18" width="0.1524" layer="91"/>
<wire x1="40.64" y1="43.18" x2="40.64" y2="48.26" width="0.1524" layer="91"/>
<junction x="40.64" y="43.18"/>
<pinref part="U1" gate="G$1" pin="VDD"/>
<wire x1="40.64" y1="48.26" x2="50.8" y2="48.26" width="0.1524" layer="91"/>
<pinref part="C2" gate="G$1" pin="P$1"/>
<wire x1="40.64" y1="48.26" x2="27.94" y2="48.26" width="0.1524" layer="91"/>
<wire x1="27.94" y1="48.26" x2="27.94" y2="38.1" width="0.1524" layer="91"/>
<junction x="40.64" y="48.26"/>
<pinref part="VDD1" gate="G$1" pin="VDD"/>
<wire x1="40.64" y1="48.26" x2="38.1" y2="48.26" width="0.1524" layer="91"/>
<wire x1="38.1" y1="48.26" x2="38.1" y2="76.2" width="0.1524" layer="91"/>
<wire x1="38.1" y1="76.2" x2="38.1" y2="81.28" width="0.1524" layer="91"/>
<wire x1="38.1" y1="76.2" x2="101.6" y2="76.2" width="0.1524" layer="91"/>
<wire x1="101.6" y1="76.2" x2="101.6" y2="45.72" width="0.1524" layer="91"/>
<junction x="38.1" y="76.2"/>
<pinref part="U1" gate="G$1" pin="CSB"/>
<wire x1="101.6" y1="45.72" x2="101.6" y2="33.02" width="0.1524" layer="91"/>
<wire x1="101.6" y1="33.02" x2="86.36" y2="33.02" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="1"/>
<wire x1="132.08" y1="45.72" x2="101.6" y2="45.72" width="0.1524" layer="91"/>
<junction x="101.6" y="45.72"/>
</segment>
<segment>
<pinref part="SJ1" gate="G$1" pin="1"/>
<pinref part="VDD2" gate="G$1" pin="VDD"/>
<wire x1="129.54" y1="71.12" x2="129.54" y2="73.66" width="0.1524" layer="91"/>
</segment>
</net>
<net name="SDA" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="SDX/SDA"/>
<wire x1="86.36" y1="58.42" x2="119.38" y2="58.42" width="0.1524" layer="91"/>
<wire x1="119.38" y1="58.42" x2="119.38" y2="43.18" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="2"/>
<wire x1="119.38" y1="43.18" x2="132.08" y2="43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="SCL" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="SCX/SCL"/>
<wire x1="86.36" y1="55.88" x2="114.3" y2="55.88" width="0.1524" layer="91"/>
<wire x1="114.3" y1="55.88" x2="114.3" y2="40.64" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="3"/>
<wire x1="114.3" y1="40.64" x2="132.08" y2="40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="SDO/SA0"/>
<wire x1="86.36" y1="53.34" x2="109.22" y2="53.34" width="0.1524" layer="91"/>
<wire x1="109.22" y1="53.34" x2="109.22" y2="66.04" width="0.1524" layer="91"/>
<pinref part="SJ1" gate="G$1" pin="2"/>
<wire x1="109.22" y1="66.04" x2="124.46" y2="66.04" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
