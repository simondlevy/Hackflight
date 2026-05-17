<img src="main.png">

# Add M2 nylon machine screws

Before soldering on any of the components, you'll need to decide how you're going to mount the PCB to your
airframe.  I've also used [VHB tape](https://www.amazon.com/dp/B007Y7GIKI?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_3),
which doesn't require screws, but for a cleaner build the PCB provides four mounting holes in the popular
26x26mm diamond configuration, for use with
[M2 nylon machine screws](https://www.amazon.com/dp/B007Y7GIKI?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_3).
You secure these screws temporarily with nuts before using a rubber grommit for vibration dampening.

# BOM, in order of soldering

Note: I recommend using Scotch tape to mask the pins you're not currently
soldering, and doing a continuity check for bridges/shorts after each step:

1. [Pololu 5V Step-Up/Step-Down Voltage Regulator S7V7F5](https://www.pololu.com/product/2119), on bottom, using 1-2mm chisel/bevel tip.

2. [28AWG silicone-insulated wire](https://www.amazon.com/BOJACK-Flexible-Silicone-Electric-Stripper/dp/B09Y89X5CV) for hover-deck, 
   protruding from bottom, using 0.2-0.8mm fine/conical tip.

3. [SHUTTLE BOARD 3.0 BMI088](https://www.digikey.com/en/products/detail/bosch-sensortec/SHUTTLE-BOARD-3-0-BMI088/14617528), on top, 
using 0.2-0.8mm fine/conical tip.  To avoid soldering a bridge (short circuit), you should solder only the four indicated
pins (3V3, GND, SDA, SCL), and in the bottom corners for added mounting security.

4. 28AWG silicone-insulates wires to 4-in-1 ESC, protruding from bottom, using 0.2-0.8mm fine/conical tip.
Most 4-in-1 ESCs come with this [connector](../../media/4-in-1-esc-wiring.jpg), or you can build one yourself
using a kit like
[this](https://www.amazon.com/Teansic-Connector-Pre-Crimped-Housing-Controller/dp/B0D5X6BY5Z/ref=sr_1_1_sspa?crid=1UNL8YQMAGJI0).

5. 220Ω resistor, on bottom, using 1-2mm chisel/bevel tip.

6. [LED](https://lighthouseleds.com/1-8mm-2mm-led-red-ultra-bright.html), on top, using 1-2mm chisel/bevel tip.
