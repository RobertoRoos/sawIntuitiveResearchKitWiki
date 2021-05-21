# Introduction

As of May 2021, the dVRK hardware and software is designed to control the first generation da Vinci, aka Classic (see [FAQ](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ) for details regarding da Vinci versions).  On the patient's side, the PSM arms are designed to use interchangeable tools depending on the task (e.g. needle driver, scissor, grasper...).  Since the da Vinci classic has been retired in 2012 it is getting harder to find tools for this model.  Fortunately, when Intuitive Surgical introduced the da Vinci S (and later Si) the tools remained **almost** unchanged so it is possible to use S/Si tools with the dVRK Classic PSMs. 

# Differences between Classic and S/Si tools

The main differences are:
* Model number: both generation of tools are identified by a 6 digits number.  Classic tool model numbers start with **400** while S/Si tool model numbers start with **420**.  For example, a classic large needle driver (LND) uses **400006** while a S/Si LND uses **420006**.
* Length: S/Si tools are longer.  We assume this is the main reason this new tool version was introduced.  If you are curious, you should look at S/Si PSM arms, they have an interesting two stage translation link for insertion.  This allows to "collapse" the arm when the tool is inserted and reduce the risk of collision between the arms outside the patient.
* Shape of the tool base plate and sterile adapter: since both the Classic and S/Si were commercially available at the same time, the tools have a different base plate to make sure a Classic tool can't be accidentally inserted on a S/Si PSM and a S/Si tool can't be inserted in a Classic PSM.  To circumvent this, one can use a modified Classic sterile adapter that can accept a S/Si tool. 

## Sterile adapter for S/Si

To use a S/Si tool, one need to use a modified Classic sterile adapter.  The modifications are a bit challenging so you should probably try to get some modified sterile adapters directly from ISI:
* Side grooves need to be deepen
* Ends of side grooves need to be cut to allow for a large tool
* Bottom lip needs to be removed
* Middle bar needs to be shaven but not too much otherwise the matting wheels might fall 

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tools/serile-adapter-S-Si-tools.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tools/serile-adapter-S-Si-tools.jpg" width="350"></a>

**Note:** these mechanical modifications are on top of the [electric modification](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware#12-sterile-adapter) that requires to short two pins on the back of the sterile adapter.

## Tool parameters

As far a we can tell the only parameter that needs to change between a Classic and a S/Si tool is the shaft length.  See for example the files `share/tool/LARGE_NEEDLE_DRIVER_400006.json` and `share/tool/LARGE_NEEDLE_DRIVER_420006.json`.  For the first joint, DH:
* Classic: `"D":  0.4162` (i.e. 41.62 cm)
* S/Si: `"D":  0.4670` (i.e. 46.7 cm, 5.08 cm longer)

