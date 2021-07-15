-- https://stackoverflow.com/questions/23068218/haskell-read-raw-keyboard-input

module GetKey where
import System.IO (stdin, hReady)

getKey :: IO [Char]
getKey = reverse <$> getKey' ""
  where getKey' chars = do
          char <- getChar
          more <- hReady stdin
          (if more then getKey' else return) (char:chars)
