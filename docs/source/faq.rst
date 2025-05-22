.. _faq:
****
FAQs
****

**Q: Which simulator or engine do I need to use to create my digital twin?**

A: You can use any simulator or engine depending on your usecase. The SDK is designed to be simulator/engine agnostic.
However, you need to setup your own simulator and engine and figure out the ros communication within the simulator/engine. 
After deciding on which simulator/engine to use, you can use add your engine's client manually in the generated files

**Q: Woudn't the private dockerized cloud add latency?**

A: In contrary to this assumption, the digital twin has shown an average latency of 19ms when tested on a campus network with multiple access points.
The latency increases when transferring from one access point to another and has an average of 300ms for areas with weak network coverage and far access points.
