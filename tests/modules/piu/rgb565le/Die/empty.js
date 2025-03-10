/*---
description: 
flags: [onlyStrict]
---*/

const one = new Content(null, { left: 0, top: 0, height: 100, width: 100, skin: new Skin({ fill: "red"}) });
const two = new Content(null, { left: 0, top: 100, height: 100, width: 100, skin: new Skin({ fill: "yellow"}) });
const three = new Content(null, { left: 0, top: 200, height: 100, width: 100, skin: new Skin({ fill: "blue"}) });

const die = new Die(null, {
	top: 0, bottom: 0, left: 0, right: 0,
	contents: [one, two]
});

new Application(null, {
	skin: new Skin({ fill: "white" }),
	contents: [ die, three ]
});
die.fill()
	.cut();
screen.checkImage("4ccc01803b9dcd9f9f0da37617f9cbf0");

die.empty()
	.cut();
screen.checkImage("810dbe035803d397a144531c4667c225");

die.empty()
	.set(25, 25, 50, 100)
	.cut();
screen.checkImage("ac08269f02b0fd07f36a77e215157552");
