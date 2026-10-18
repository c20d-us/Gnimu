# Installing the carrier PCB in a project box

This page walks through fitting a loaded [Gnimu carrier PCB](build-nrf52840.md) into an ABS project box with a LiPo cell and a slide switch to make a finished portable unit.

---

## What you need

- A loaded Gnimu carrier PCB (XIAO nRF52840 Sense, buck-boost module, GNSS module, resistors, and JST leads). See [Building the XIAO nRF52840 device](build-nrf52840.md).
- An ABS [project box](https://www.amazon.com/dp/B0BQYPKRQS), 80x50x26 mm exterior, 75x45x22 mm interior
- A latching [Single Pole Double Throw (1P2T) slide switch](https://www.amazon.com/dp/B0BWMS64PR) with mounting flange
- A [3.7V LiPo cell](https://www.amazon.com/dp/B0DPZVBKMY) (build below uses an 803040, 1000mAh)
- 4x M2.5 hex standoffs, screws, and washers to mount the PCB
- Drill press and bits, small file or chisel, rotary tool or sharp hobby knife for the notches

<p align="center">
  <img src="../images/PCB-build/00-gnimu-carrier-pcb-loaded.jpeg" alt="The loaded Gnimu carrier PCB" width="520"><br>
  <sub>The loaded carrier PCB, with the battery, switch, and GNSS connectors still loose.</sub>
</p>

---

## 1. Prepare the box

### 1.1 Start with the bare box

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/01-project-box.jpeg"><img src="../images/PCB-build/thumbs/01-project-box.jpeg" alt="The project box and lid" width="300"></a>
    </td>
    <td valign="top">
        The box as it comes. Which end gets the switch and which gets the USB notch is not critical. I put the USB at the end with the lid-opening notches.<br><br>
        <b>Note:</b> the mounting holes in the PCB are designed to line up exactly with the standoffs in this particular brand of box. If you use a different brand of box, the standoffs/holes may not exactly line up. Check before you start.
    </td>
  </tr>
</table>

### 1.2 Drill the standoff holes

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/02-project-box-drilled.jpeg"><img src="../images/PCB-build/thumbs/02-project-box-drilled.jpeg" alt="Box base with four drilled holes" width="300"></a>
    </td>
    <td valign="top">
        Drill out the plastic standoff holes from the inside with a 7/64" bit. Use the existing holes in the plastic standoffs to center the bit as you drill through.
    </td>
  </tr>
</table>

### 1.3 Clear the interior

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/03-project-box-shaved.jpeg"><img src="../images/PCB-build/thumbs/03-project-box-shaved.jpeg" alt="Shaving the molded interior features with a chisel" width="300"></a>
    </td>
    <td valign="top">
        After you drill out the plastic standoffs, use a sharp chisel to cut off what is left of the standoffs. Make sure the corners of the box have no remaining bits of standoff and are as flat as you can make them.
    </td>
  </tr>
</table>

### 1.4 Prep the lid

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/ESP32/GNSS-and-lid.jpeg"><img src="../images/PCB-build/thumbs/GNSS-and-lid.jpeg" alt="GNSS module with mounting hole in the lid" width="300"></a>
      <br><br>
      <a href="../images/PCB-build/04-led-and-reset-hole.jpeg"><img src="../images/PCB-build/thumbs/04-led-and-reset-hole.jpeg" alt="Lid with the LED and reset holes, a hot-glue gun, and a utility knife blade for flattening the glue" width="300"></a>
      <br><br>
      <a href="../images/PCB-build/05-gnss-in-lid.jpeg"><img src="../images/PCB-build/thumbs/05-gnss-in-lid.jpeg" alt="Finished lid with the GNSS module in its opening and the LED showing through" width="300"></a>
    </td>
    <td valign="top">
        Create a square hole in one end of the lid to accomodate the GNSS's patch antenna. The M100-5883 has a small flange around the patch antenna that is perfect for holding the GNSS to the lid with a few dabs of superglue. Start with a large-ish drilled hole in one end of the lid, then shave plastic away with your utility knife until the GNSS just fits in the opening. You can notch the edge side to allow visibility to the LEDs.<br><br>
        You may want to drill two small holes in the XIAO end of the lid for the LED and reset button. Use the mounted XIAO in the box to determine where these two need to go. 3/32" holes should be plenty big enough.<br><br>
        <b>Optional:</b> put a dab of hot-glue in the LED hole for a DIY light-pipe effect. Hold the lid tight against a utility knife blade while you put the hot glue in the hole so that the top side of the glue spot turns out flat.<br><br>
        <b>Optional:</b> If you don't want to cut a hole in the lid, there should be *just* enough room above the PCB to affix the GNSS to the underside of the lid with some double-sided tape. This shouldn't affect signal reception at all.
    </td>
  </tr>
</table>

---

## 2. Cut the openings

### 2.1 USB-C notch

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/06-usb-plug-notch.jpeg"><img src="../images/PCB-build/thumbs/06-usb-plug-notch.jpeg" alt="Box end wall with a small notch" width="300"></a>
      <br><br>
      <a href="../images/PCB-build/07-usb-plug-notch.jpeg"><img src="../images/PCB-build/thumbs/07-usb-plug-notch.jpeg" alt="PCB test-fit with the USB-C port in the notch" width="300"></a>
    </td>
    <td valign="top">

Notch the top edge of one end of the box to accomodate the XIAO's USB plug. The notch should be roughly 9mm wide and 3.5mm deep. You can use the board as a reference to mark the left and right edges, and then carefully cut away plastic to make the notch. It's not critical to get it exactly right at this step, as you can adjust it a bit later. Just go slow and remember that you can always take more off, but you can't put any back.

You'll need to notch the vertical flange on the underside of the lid to match where the USB port lands.

</td>
</tr>
</table>

### 2.2 Switch opening

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/08-switch-starter-hole.jpeg"><img src="../images/PCB-build/thumbs/08-switch-starter-hole.jpeg" alt="Starter hole drilled for the switch" width="300"></a>
      <br><br>
      <a href="../images/PCB-build/09-switch-hole-enlarged.jpeg"><img src="../images/PCB-build/thumbs/09-switch-hole-enlarged.jpeg" alt="Switch opening enlarged to a rectangle" width="300"></a>
    </td>
    <td valign="top">

Start the opening for the switch by drilling a 3/16" hole in the opposite end of the box from the USB notch. You want the 3/16" hole's bottom edge to be right at the floor of the box. Once you have the hole drilled, use your utility knife to open up the hole to a rectangle for the switch body. You want the rectangle to be just big enough for the switch body to fit through, so again, go slow and take off small amounts at a time, checking fit with the switch often.

**Note:** I chose to put the switch in the end of the box, but realistically there is room to put the switch on either side of the box at the end opposite the battery. Pick a location that suits your taste. Just be mindful to not interfere with the locations of the standoffs. Wherever you put it, I recommend have the switch as low as possible to ensure that it's clear of the PCB.

</td>
</tr>
</table>

### 2.2 Switch mounting screws

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/10-switch-screwholes-drilled.jpeg"><img src="../images/PCB-build/thumbs/10-switch-screwholes-drilled.jpeg" alt="Switch mounted in the box end wall" width="300"></a>
    </td>
    <td valign="top">

Once you have the hole for the switch big enough, put the switch in the hole and use a drill press to drill through the mounting holes. This should be done with a 7/64" drill bit.

</td>
</tr>
</table>

---

## 3. Install the hardware

### 3.1 Switch and standoffs

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/11-switch-and-standoffs.jpeg"><img src="../images/PCB-build/thumbs/11-switch-and-standoffs.jpeg" alt="Switch wired with a connector, and four standoffs in the box" width="300"></a>
      <br><br>
      <a href="../images/PCB-build/12-standoff-screws.jpeg"><img src="../images/PCB-build/thumbs/12-standoff-screws.jpeg" alt="Underside of the box with standoff screws" width="300"></a>
    </td>
    <td valign="top">

Feed the switch's JST pigtail through the hole, then use M2.5 screws and nuts to fasten the switch to the box. The hex nuts should fit right up against the switch body as you tighten the screws.

Feed an M2.5x5mm screw through each hole in the bottom of the box, put one 1mm washer over each, and then affix a 10mm hex standoff. These should be snugged down well but not overtightened so that you don't strip out the standoffs. If you want, after fully assembling the box you can disassemble it and then put the standoffs back in with a bit of glue to hold them permanently.

</td>
</tr>
</table>

### 3.2 Prepare the battery bay

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/13-battery-prep.jpeg"><img src="../images/PCB-build/thumbs/13-battery-prep.jpeg" alt="Battery bay prepared with a pad of tape" width="300"></a>
    </td>
    <td valign="top">

Double-check the cell for damage before installing. Put some double-sided tape at the USB end of the box to hold the battery. I use 3M squares that hold well but that can be removed if needed.

</td>
</tr>
</table>

### 3.4 Install the battery

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/14-battery-installed.jpeg"><img src="../images/PCB-build/thumbs/14-battery-installed.jpeg" alt="LiPo cell installed in the box" width="300"></a>
    </td>
    <td valign="top">

Mount the battery to the bottom of the box as shown, with the pigtail going toward the center. If you're using an 8mm or thinner battery there should be ample room between the bottom of the PCB and the battery once everything is in place. You can go with a 5mm or 6mm thick cell if you want extra clearance, but you will likely give up some capacity.

</td>
</tr>
</table>

---

## 4. Install the PCB

### 4.1 Connect the cables

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/15-under-board.jpeg"><img src="../images/PCB-build/thumbs/15-under-board.jpeg" alt="PCB tilted up while the underside connectors are plugged in" width="300"></a>
    </td>
    <td valign="top">

Make sure the switch is off (switch to yellow wire in my build), then plug in the battery lead (2-wire JST) and the switch lead (4-wire JST with one wire clipped off). Once the JST plugs are connected, carefully tuck the battery and switch wires into the box and settle the PCB down onto the standoffs. Make sure nothing is getting pinched.

</td>
</tr>
</table>

### 4.2 Secure the board

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/16-board-secured.jpeg"><img src="../images/PCB-build/thumbs/16-board-secured.jpeg" alt="PCB screwed down onto the standoffs" width="300"></a>
    </td>
    <td valign="top">

Use one M2.5x5mm screw at each corner of the PCB to secure it to the standoffs. Before you tighten everything down, make sure the the board sits flat on the standoffs - you may need to make the USB plug notch a bit deeper for this. There should be just a very tiny space visible below the bottom of the USB plug when the board is flat on the standoffs. Unlike with the bottom ends of the standoffs, DO NOT use glue on the PCB mounting screws. You may need to remove them later to swap a battery.

Once everything is secured, connect the GNSS module's JST plug, connect the JST plug for the XIAO's battery lead, and flip the switch on. If you've connected it all correctly, the device should power up. Turn the switch back off, and carefully attach the lid to the box, making sure you don't pinch any wires.

</td>
</tr>
</table>

### 4.3 Close the box

<table>
  <tr>
    <td valign="top" width="320">
      <a href="../images/PCB-build/17-complete-device.jpeg"><img src="../images/PCB-build/thumbs/17-complete-device.jpeg" alt="The finished device with the GNSS module on the lid" width="300"></a>
    </td>
    <td valign="top">

You're all done!

</td>
</tr>
</table>

---

## Checklist

- [ ] USB-C port is reachable through the notch
- [ ] Switch moves freely and clicks on and off
- [ ] Battery is clear of the PCB underside and the standoffs
- [ ] GNSS lead has slack to open the lid
- [ ] Lid closes without pinching any wires
