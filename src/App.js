import logo from './logo.svg';
import './App.css';

function App() {
  const name = "Xenia";
  let varname = "";
  if (name == "Xenia") {
    varname = (<p> Hey my name is xenia </p>);
  }
  const buttonclick = () => {
    console.log("I clicked the button")
  }

  return (
    <div className="App">
     <header className="App-header">
        <img src={logo} className="App-logo" alt="logo" />
        <p>
          Edit <code>src/App.js</code> and save to reload.
        </p>
      <p> Hey my name is {name}</p>
      {varname}
      <button onClick={buttonclick}>I am a button</button>
        <a
          className="App-link"
          href="https://reactjs.org"
          target="_blank"
          rel="noopener noreferrer"
        >
          Learn React
        </a>
      </header>
    </div>
  );
}
export default App;