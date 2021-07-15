import System.IO (stdin, hSetEcho, hSetBuffering,  BufferMode( NoBuffering ) )
import GetKey

main :: IO ()
main = do
  hSetBuffering stdin NoBuffering
  hSetEcho stdin False
  key <- getKey
  putStrLn key
  main
