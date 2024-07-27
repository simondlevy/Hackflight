<p align="center"> 
<img src="../media/lambda.png" width=600>
</p>

# &ldquo;LambdaFlight&rdquo;: Hackflight meets Haskell

To experiment with using Hackflight for flight control in Haskell, you'll first
need to install [Haskell](https://www.haskell.org/) and [NASA
Copilot](https://copilot-language.github.io) (the &ldquo;secret sauce&rdquo;
that allows you to compile Haskell code to a form suitable for running on a
flight controller.)  I was able install Copilot via:

```
cabal install copilot
cabal install copilot-c99
```

Next, follow these [directions](../webots) for installing Webots and flying a simulated quadcopter.

Finally, from the hackflight main directory, do the following:

```
cd webots/controllers/haskell
make
cd ../..
make haskell
```





