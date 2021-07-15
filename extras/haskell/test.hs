import System.IO (stdin, hSetEcho, hSetBuffering)
import System.IO( BufferMode( NoBuffering ) )

import GetKey

main = do
  hSetBuffering stdin NoBuffering
  hSetEcho stdin False
  key <- getKey
  putStrLn key
  main
