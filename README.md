# ailedeau
Simulation, design and control of l'aile d'eau


L'aile d'eau means water wing in french.

It is the name given by Luc Armant to its sailing engine. 
https://www.augredelair.fr/wp-content/uploads/2015/01/luc_armant_ailedeau.pdf

The first aim of this directory is to reproduce the results of the original paper.
And maybe build on them.

An excel based velocity prediction program was created by Luc and shared in Annex D page 129.

The first step was to reproduce this calculation sheet.
This was done here in a google sheet.
The result are slightly different due to the fact that some approximation were used for some constants.
https://docs.google.com/spreadsheets/d/12voMISq12KGp8djZCUUUPkHIgdLkBO8MEXWyJg5wJbI/edit?pli=1

The next step is to understand the different formula and to code them in a proper programming language.
Unfortunately, there are little details on the derivation of some formula in the reference paper.
And this approach was stopped for now

The goal is to get to a live version of the simulator than can be used in a web browser.

A dynamic simulator is also developed.

## 🛠️ Installation
You can download the code directly from GitHub or using git:

```bash
git clone git@github.com:baptistelabat/ailedeau.git
```

To install the required dependencies, follow the steps below:

1. Install `uv` by following the official [installation guide](https://docs.astral.sh/uv/getting-started/installation).
2. Alternatively, run the command from root of repository:  
   ```bash
   cd ailedeau
   make install-uv
  
   sudo apt-get install -y libxcb-cursor-dev

   ```
## Developer guidelines
Google docstring are used
Please use typing
Please try to get a maximal code coverage, first by deleting unused code
