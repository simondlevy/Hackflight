import System.IO (stdin, hSetEcho, hSetBuffering,  hReady, BufferMode( NoBuffering ) )

getKey :: IO [Char]
getKey = reverse <$> getKey' ""
  where getKey' chars = do
          char <- getChar
          more <- hReady stdin
          (if more then getKey' else return) (char:chars)


main :: IO ()
main = do
  hSetBuffering stdin NoBuffering
  hSetEcho stdin False
  key <- getKey
  putStrLn key
  main
