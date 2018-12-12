# Introduction

This page lists some currently available datasets for robotics surgery, sorted by the information they include. It also lists data that we recommend to be collected for groups running user studies going forward. Please contact us if you would like your dataset to be included on this page. 

# Dataset requirements
### Data description 
In order to easily share data from studies between groups, we recommend the following data to be reported for every dataset. 
* Clinical Application (Suturing? Knot tying? Cutting? Etc.)
* Data Types (video, position, velocity, torque, current, multiple...)
* Configuration Data:  camera intrinsic (and extrinsic) parameters, if available, base frames (e.g., for PSMs, ECM)
* Kinematics parameters - include JSON dVRK config files
* Environment (phantom, animal, patient...)
* Users: experts, non-experts, mix (be careful to avoid personal data)
* System (dVRK, da Vinci (with permission from ISI), simulated, Raven, etc.)
* Trial size:  number of participants and number of trials per participant
* Data Size: megabytes
* Background Information: e.g., part of clinical trial
* Citation: credit if data is used
* Version of software/firmware the data was collected on

### Data/privacy protection
Please consider your institution's and country's rules regarding data privacy and company policies before uploading any data. It is up to each individual group to obtain permission and user consent to share the data publicly. Time stamps should include an offset to further anonymize studies. Consider asking for the following in all consent forms before conducting user studies to maximize usability of the data:
* Capture and use data for any future research (as opposed to a single study)
* Permission to share anonymized data with other sites
* Permission to store data indefinitely

# Datasets

|Dataset| Clinical application | Data type | Environment | Users | System | 
|-------|----------------------|-----------|-------------|-------|--------|
|[JIGSAWS](https://cirl.lcsr.jhu.edu/research/hmm/datasets/jigsaws_release/)| Suturing, Knot-Tying, Needle-Passing | Video, Kinematics, Phase Annotation | Phantom | Mixed (8)| da Vinci Research API |